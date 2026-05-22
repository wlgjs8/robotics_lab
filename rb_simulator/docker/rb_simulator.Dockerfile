FROM python:3.13-slim

ENV PYTHONDONTWRITEBYTECODE=1 \
    PYTHONUNBUFFERED=1 \
    PYTHONPATH=/workspace/rb_simulator/src

RUN apt-get update \
    && apt-get install -y --no-install-recommends socat \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /workspace
COPY rb_simulator /workspace/rb_simulator
COPY rb_simulator/docker/rb_simulator_entrypoint.sh /usr/local/bin/rb_simulator_entrypoint.sh
RUN chmod +x /usr/local/bin/rb_simulator_entrypoint.sh

ENTRYPOINT ["/usr/local/bin/rb_simulator_entrypoint.sh"]
CMD ["--config", "rb_simulator/config/dual_rb3_730e.yaml"]
