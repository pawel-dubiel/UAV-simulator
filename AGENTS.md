# Repository Guidelines

## Project Structure & Module Organization
- Root module (`go.mod`). Core code in `internal/sim/` package.
- Key files: `internal/sim/simulator.go` (loop), `internal/sim/drone.go` (physics), `internal/sim/renderer.go` (OpenGL), `internal/sim/camera.go`, `internal/sim/input.go`, `internal/sim/math.go`.
- App entry: `main.go` (window setup, flags, vsync).
- Tests in `test/` using `sim_test` and importing `drone-simulator/internal/sim`.

## Build, Test, and Development Commands
- `go mod tidy`: Sync dependencies.
- `go build -o drone-simulator .`: Build local binary (ignored by Git).
- `go run .`: Run the simulator (decoupled render/physics by default).
- `go test ./...`: Run all unit tests.
- `go test -cover ./...`: Run tests with coverage.
- `go vet ./...`: Static checks for common issues.
 - Legacy loop (if needed): `go run . -decoupled=false`

## Coding Style & Naming Conventions
- Use `go fmt ./...` before committing; standard Go formatting (tabs, imports grouped).
- Package names lower-case and short; exported identifiers in `UpperCamelCase`, unexported in `lowerCamelCase`.
- Filenames use `snake_case.go`; tests mirror source names as `*_test.go` (e.g., `swarm_diagnostics_test.go`).
- Keep functions small and pure where possible; separate rendering from physics/math logic.

## Testing Guidelines
- Framework: standard `testing` package; table-driven tests encouraged.
- Scope: prioritize math/physics (`math.go`, `drone.go`, swarm logic) which run headless.
- Naming: `TestXxx` in files named like `module_name_test.go`.
- Commands: `go test ./... -run <Name>` to target specific tests; add `-v` for verbose.
- Avoid opening OpenGL windows in tests; keep tests deterministic and CI-friendly.

## Commit & Pull Request Guidelines
- Commits follow Conventional Commits (seen in history): `feat:`, `refactor:`, etc. Use imperative mood and concise scope.
- PRs: include purpose, linked issues, and screenshots/GIFs for UI changes. List test coverage touched and manual steps to verify.
- Requirements before review: `go fmt`, `go vet`, `go test ./...` all pass. Do not commit binaries (`/drone-simulator`, `/test_triangle`) or local caches.

## Security & Configuration Tips
- Requirements: Go 1.19+ and an OpenGL 4.1–capable GPU/driver. GLFW is pulled via Go modules.
- Keep secrets out of the repo (`.env*` is ignored). Avoid vendoring; `vendor/` is ignored here.

## Maintainer Expertise
- The project is maintained by an experienced game and simulation developer with a physics degree. Complex models (dynamics, control, and numerical methods) are welcome and reviewed with domain rigor.

## Headless Mode
- Run deterministic updates without a window for CI/benchmarks:
  - `./drone-simulator -headless -steps=10000 -ups=240`
  - `./drone-simulator -headless -duration=2s`
- Flags: `-arm` (auto-arm), `-ups` (updates/sec). Prefer `-steps` for repeatability in CI.
