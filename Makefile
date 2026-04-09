# Default: run the full test suite (requires: uv sync --group dev)
.PHONY: test
test:
	uv run pytest

.PHONY: test-verbose
test-verbose:
	uv run pytest -v

# Hardware: set AL5D_SERIAL if not using /dev/ttyUSB0
.PHONY: test-integration
test-integration:
	RUN_INTEGRATION_TESTS=1 uv run pytest tests/integration -v

.PHONY: test-integration-motion
test-integration-motion:
	RUN_INTEGRATION_TESTS=1 RUN_INTEGRATION_MOTION=1 uv run pytest tests/integration -v
