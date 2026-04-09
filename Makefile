# Default: run the full test suite (requires: uv sync --group dev)
.PHONY: test
test:
	uv run pytest

.PHONY: test-verbose
test-verbose:
	uv run pytest -v
