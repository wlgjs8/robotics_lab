FROM python:3.13-slim

ENV PYTHONDONTWRITEBYTECODE=1 \
    PYTHONUNBUFFERED=1 \
    PYTHONPATH=/workspace/rb_simulator/src

WORKDIR /workspace
COPY rb_simulator /workspace/rb_simulator

ENTRYPOINT ["python", "-m", "rbsim"]
CMD ["--config", "rb_simulator/config/left_rb3_730e.yaml"]
