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

RUN set -eux; \
    if getent group "${USER_GID}" >/dev/null; then \
        GROUP_NAME="$(getent group "${USER_GID}" | cut -d: -f1)"; \
    else \
        groupadd --gid "${USER_GID}" "${USERNAME}"; \
        GROUP_NAME="${USERNAME}"; \
    fi; \
    if id -u "${USERNAME}" >/dev/null 2>&1; then \
        true; \
    elif getent passwd "${USER_UID}" >/dev/null; then \
        EXISTING_USER="$(getent passwd "${USER_UID}" | cut -d: -f1)"; \
        usermod --login "${USERNAME}" --home "/home/${USERNAME}" --move-home "${EXISTING_USER}"; \
    else \
        useradd --uid "${USER_UID}" --gid "${GROUP_NAME}" --create-home --home-dir "/home/${USERNAME}" "${USERNAME}"; \
    fi; \
    usermod --gid "${GROUP_NAME}" "${USERNAME}"; \
    mkdir -p /tmp/runtime-simulator /home/simulator/IC-IA-webots_loop; \
    chown -R "${USERNAME}:${USER_GID}" /tmp/runtime-simulator /home/simulator

WORKDIR /home/simulator/IC-IA-webots_loop

COPY --chown=${USERNAME}:${USER_GID} . .

RUN make -C controllers/my_controller_Micael \
    && mkdir -p simulation_results \
    && chown -R "${USERNAME}:${USER_GID}" /home/simulator/IC-IA-webots_loop

USER ${USERNAME}

CMD ["python3", "Simulation_loop.py"]
