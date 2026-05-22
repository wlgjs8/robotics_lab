FROM python:3.13-slim

ENV PYTHONDONTWRITEBYTECODE=1 \
    PYTHONUNBUFFERED=1 \
    RB_GUI_HOST=0.0.0.0 \
    RB_GUI_PORT=8080 \
    RB_GUI_STATE_BIND=0.0.0.0 \
    RB_GUI_STATE_PORT=50110 \
    RB_GUI_COMMAND_HOST=rb_servo_server \
    RB_GUI_COMMAND_PORT=50010 \
    RB_GUI_DESCRIPTIONS_DIR=/app/descriptions \
    RB_GUI_ROBOT_URDF=/app/descriptions/urdf/rb3_730e.urdf \
    RB_GUI_STAND_MESH=/app/descriptions/meshes/stands/dual_rb3_730e/dual_rb3_730e_stand_ver3.stl

WORKDIR /app
COPY rb_gui/pyproject.toml /app/pyproject.toml
COPY rb_gui/rb_servo_gui /app/rb_servo_gui
RUN pip install --no-cache-dir .
COPY rb_servo_server/descriptions /app/descriptions
EXPOSE 8080/tcp 50110/udp
CMD ["python", "-m", "rb_servo_gui.app"]
