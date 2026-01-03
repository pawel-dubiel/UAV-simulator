//go:build darwin && !test
// +build darwin,!test

package avaudio

/*
#cgo darwin CFLAGS: -x objective-c -fobjc-arc -Wno-deprecated-declarations
#cgo darwin LDFLAGS: -framework AVFoundation -framework Foundation
#include <stdlib.h>
#include <string.h>

#import <AVFoundation/AVFoundation.h>

@interface AVAudioSim : NSObject
@property(nonatomic, strong) AVAudioEngine *engine;
@property(nonatomic, strong) AVAudioEnvironmentNode *environment;
@property(nonatomic, strong) NSMutableArray<AVAudioPlayerNode *> *players;
@property(nonatomic, strong) NSMutableArray<AVAudioUnitVarispeed *> *varispeeds;
@property(nonatomic, strong) AVAudioPCMBuffer *buffer;
@property(nonatomic, assign) int droneCount;
@end

@implementation AVAudioSim
@end

static void avaudio_set_error(char **err, const char *msg) {
	if (!err || *err) {
		return;
	}
	if (!msg) {
		msg = "unknown error";
	}
	*err = strdup(msg);
}

static void *avaudio_create(int droneCount, int sampleRate, const float *samples, int frameCount, char **err) {
	if (droneCount <= 0) {
		avaudio_set_error(err, "audio init: drone count must be > 0");
		return NULL;
	}
	if (sampleRate <= 0 || frameCount <= 0 || samples == NULL) {
		avaudio_set_error(err, "audio init: invalid sample data");
		return NULL;
	}

	@autoreleasepool {
		AVAudioSim *sim = [AVAudioSim new];
		sim.engine = [AVAudioEngine new];
		sim.environment = [AVAudioEnvironmentNode new];
		sim.players = [NSMutableArray arrayWithCapacity:droneCount];
		sim.varispeeds = [NSMutableArray arrayWithCapacity:droneCount];
		sim.droneCount = droneCount;

		AVAudioEnvironmentDistanceAttenuationParameters *atten = sim.environment.distanceAttenuationParameters;
		atten.referenceDistance = 2.0;
		atten.rolloffFactor = 1.0;
		atten.maximumDistance = 60.0;

		[sim.engine attachNode:sim.environment];

		AVAudioFormat *format = [[AVAudioFormat alloc] initWithCommonFormat:AVAudioPCMFormatFloat32 sampleRate:sampleRate channels:1 interleaved:NO];
		[sim.engine connect:sim.environment to:sim.engine.mainMixerNode format:nil];

		sim.buffer = [[AVAudioPCMBuffer alloc] initWithPCMFormat:format frameCapacity:(AVAudioFrameCount)frameCount];
		sim.buffer.frameLength = (AVAudioFrameCount)frameCount;
		memcpy(sim.buffer.floatChannelData[0], samples, sizeof(float) * (size_t)frameCount);

		for (int i = 0; i < droneCount; i++) {
			AVAudioPlayerNode *player = [AVAudioPlayerNode new];
			AVAudioUnitVarispeed *varispeed = [AVAudioUnitVarispeed new];
			player.volume = 0.0;
			player.renderingAlgorithm = AVAudio3DMixingRenderingAlgorithmHRTF;
			varispeed.rate = 1.0;

			[sim.engine attachNode:player];
			[sim.engine attachNode:varispeed];
			[sim.engine connect:player to:varispeed format:format];
			[sim.engine connect:varispeed to:sim.environment format:format];

			[player scheduleBuffer:sim.buffer atTime:nil options:AVAudioPlayerNodeBufferLoops completionHandler:nil];
			[sim.players addObject:player];
			[sim.varispeeds addObject:varispeed];
		}

		NSError *error = nil;
		if (![sim.engine startAndReturnError:&error]) {
			const char *msg = error ? error.localizedDescription.UTF8String : "audio init: AVAudioEngine start failed";
			avaudio_set_error(err, msg);
			return NULL;
		}
		for (AVAudioPlayerNode *player in sim.players) {
			[player play];
		}

		return (__bridge_retained void *)sim;
	}
}

static int avaudio_set_listener(void *ctx, float x, float y, float z, float yawDeg, float pitchDeg, float rollDeg, char **err) {
	if (!ctx) {
		avaudio_set_error(err, "audio update: context is nil");
		return 1;
	}
	@autoreleasepool {
		AVAudioSim *sim = (__bridge AVAudioSim *)ctx;
		sim.environment.listenerPosition = AVAudioMake3DPoint(x, y, z);
		AVAudio3DAngularOrientation orient;
		orient.yaw = yawDeg;
		orient.pitch = pitchDeg;
		orient.roll = rollDeg;
		sim.environment.listenerAngularOrientation = orient;
		return 0;
	}
}

static int avaudio_set_source(void *ctx, int idx, float x, float y, float z, float gain, float rate, char **err) {
	if (!ctx) {
		avaudio_set_error(err, "audio update: context is nil");
		return 1;
	}
	@autoreleasepool {
		AVAudioSim *sim = (__bridge AVAudioSim *)ctx;
		if (idx < 0 || idx >= sim.droneCount) {
			avaudio_set_error(err, "audio update: source index out of range");
			return 1;
		}
		AVAudioPlayerNode *player = sim.players[(NSUInteger)idx];
		AVAudioUnitVarispeed *varispeed = sim.varispeeds[(NSUInteger)idx];
		player.position = AVAudioMake3DPoint(x, y, z);
		player.volume = gain;
		if (rate < 0.1f) {
			rate = 0.1f;
		}
		if (rate > 3.0f) {
			rate = 3.0f;
		}
		varispeed.rate = rate;
		return 0;
	}
}

static void avaudio_destroy(void *ctx) {
	if (!ctx) {
		return;
	}
	@autoreleasepool {
		AVAudioSim *sim = (__bridge_transfer AVAudioSim *)ctx;
		[sim.engine stop];
		sim.engine = nil;
		sim.environment = nil;
		[sim.players removeAllObjects];
		[sim.varispeeds removeAllObjects];
		sim.buffer = nil;
	}
}

static void avaudio_free_error(char *err) {
	if (err) {
		free(err);
	}
}
*/
import "C"

import (
	"errors"
	"fmt"
	"unsafe"
)

type System struct {
	ptr        unsafe.Pointer
	droneCount int
}

func NewSystem(droneCount int, sampleRate int, samples []float32) (*System, error) {
	if droneCount <= 0 {
		return nil, errors.New("audio init: drone count must be > 0")
	}
	if sampleRate <= 0 {
		return nil, errors.New("audio init: sample rate must be > 0")
	}
	if len(samples) == 0 {
		return nil, errors.New("audio init: sample data is empty")
	}
	var cErr *C.char
	ptr := C.avaudio_create(C.int(droneCount), C.int(sampleRate), (*C.float)(unsafe.Pointer(&samples[0])), C.int(len(samples)), &cErr)
	if cErr != nil {
		defer C.avaudio_free_error(cErr)
		return nil, fmt.Errorf("audio init: %s", C.GoString(cErr))
	}
	if ptr == nil {
		return nil, errors.New("audio init: AVAudioEngine returned nil context")
	}
	return &System{ptr: ptr, droneCount: droneCount}, nil
}

func (s *System) DroneCount() int {
	if s == nil {
		return 0
	}
	return s.droneCount
}

func (s *System) SetListener(x, y, z, yawDeg, pitchDeg, rollDeg float64) error {
	if s == nil || s.ptr == nil {
		return errors.New("audio update: system not initialized")
	}
	var cErr *C.char
	rc := C.avaudio_set_listener(s.ptr, C.float(x), C.float(y), C.float(z), C.float(yawDeg), C.float(pitchDeg), C.float(rollDeg), &cErr)
	if cErr != nil {
		defer C.avaudio_free_error(cErr)
		return fmt.Errorf("audio update: %s", C.GoString(cErr))
	}
	if rc != 0 {
		return errors.New("audio update: listener update failed")
	}
	return nil
}

func (s *System) SetSource(idx int, x, y, z float64, gain float32, rate float32) error {
	if s == nil || s.ptr == nil {
		return errors.New("audio update: system not initialized")
	}
	var cErr *C.char
	rc := C.avaudio_set_source(s.ptr, C.int(idx), C.float(x), C.float(y), C.float(z), C.float(gain), C.float(rate), &cErr)
	if cErr != nil {
		defer C.avaudio_free_error(cErr)
		return fmt.Errorf("audio update: %s", C.GoString(cErr))
	}
	if rc != 0 {
		return errors.New("audio update: source update failed")
	}
	return nil
}

func (s *System) Close() {
	if s == nil || s.ptr == nil {
		return
	}
	C.avaudio_destroy(s.ptr)
	s.ptr = nil
}
