FROM ubuntu:24.04

ARG DEBIAN_FRONTEND=noninteractive
ARG USERNAME=simulator
ARG USER_UID=1000
ARG USER_GID=1000

ENV WEBOTS_HOME=/usr/local/webots \
    PYTHONUNBUFFERED=1 \
    QT_QPA_PLATFORM=xcb \
    XDG_RUNTIME_DIR=/tmp/runtime-simulator

RUN apt-get update \
    && apt-get install -y --no-install-recommends \
        ca-certificates \
        curl \
        ffmpeg \
        gnupg \
        libgl1 \
        libxcb-cursor0 \
        procps \
        python3 \
        xauth \
        xvfb \
        build-essential \
    && curl -fsSL https://cyberbotics.com/Cyberbotics.asc \
        | gpg --dearmor -o /usr/share/keyrings/cyberbotics.gpg \
    && echo "deb [arch=amd64 signed-by=/usr/share/keyrings/cyberbotics.gpg] https://cyberbotics.com/debian/ binary-amd64/" \
        > /etc/apt/sources.list.d/cyberbotics.list \
    && apt-get update \
    && apt-get install -y --no-install-recommends webots=2025a \
    && rm -rf /var/lib/apt/lists/*

RUN groupadd --gid "${USER_GID}" "${USERNAME}" \
    && useradd --uid "${USER_UID}" --gid "${USER_GID}" --create-home "${USERNAME}" \
    && mkdir -p /tmp/runtime-simulator /home/simulator/IC-IA-webots_loop \
    && chown -R "${USERNAME}:${USERNAME}" /tmp/runtime-simulator /home/simulator

WORKDIR /home/simulator/IC-IA-webots_loop

COPY --chown=${USERNAME}:${USERNAME} . .

RUN mkdir -p simulation_results \
    && chown -R "${USERNAME}:${USERNAME}" /home/simulator/IC-IA-webots_loop

USER ${USERNAME}

CMD ["python3", "Simulation_loop.py"]
