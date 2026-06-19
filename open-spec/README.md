# Open Spec

This folder contains human-readable design specifications for simulator behavior before implementation.

Specs should make behavior, boundaries, configuration, and test expectations explicit. Missing required parameters should fail fast with clear errors rather than silently using defaults.

## Structure

- `communication/` - Drone-to-drone communication models and contracts.

## Process

1. Define the behavior and exclusions.
2. Define explicit configuration and failure cases.
3. Define API boundaries before implementation.
4. Define deterministic tests that prove the behavior.
5. Update the spec when implementation changes the contract.
