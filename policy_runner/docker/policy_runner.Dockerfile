FROM python:3.13-slim AS runtime

ENV PYTHONDONTWRITEBYTECODE=1 \
    PYTHONUNBUFFERED=1

RUN apt-get update \
    && apt-get install -y --no-install-recommends \
       libhidapi-hidraw0 \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /app
COPY policy_runner/pyproject.toml /app/policy_runner/pyproject.toml
COPY policy_runner/policy_runner /app/policy_runner/policy_runner
COPY policy_runner/config /app/policy_runner/config
COPY policy_runner/__main__.py /app/policy_runner/__main__.py
COPY calibration /app/calibration
RUN pip install --no-cache-dir "/app/policy_runner[spacemouse]"

ENTRYPOINT ["policy-runner"]

FROM pytorch/pytorch:2.12.0-cuda13.0-cudnn9-runtime AS cuda-ml

ENV PYTHONDONTWRITEBYTECODE=1 \
    PYTHONUNBUFFERED=1

RUN apt-get update \
    && apt-get install -y --no-install-recommends \
       libhidapi-hidraw0 \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /app
COPY policy_runner/pyproject.toml /app/policy_runner/pyproject.toml
COPY policy_runner/policy_runner /app/policy_runner/policy_runner
COPY policy_runner/config /app/policy_runner/config
COPY policy_runner/__main__.py /app/policy_runner/__main__.py
COPY calibration /app/calibration
RUN python -m pip install --break-system-packages --no-cache-dir "/app/policy_runner[spacemouse,ml]"

ENTRYPOINT ["policy-runner"]
