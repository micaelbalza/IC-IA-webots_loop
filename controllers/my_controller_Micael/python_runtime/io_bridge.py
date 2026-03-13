import json
import socket
import time
from pathlib import Path


class MatlabCompatibleClient:
    def __init__(self, host="localhost", port=12345, timeout_s=120.0):
        self.host = host
        self.port = port
        self.timeout_s = timeout_s
        self.sock = self._connect()

    def _connect(self):
        start = time.time()
        while time.time() - start < self.timeout_s:
            try:
                sock = socket.create_connection((self.host, self.port), timeout=5.0)
                sock.settimeout(5.0)
                return sock
            except OSError:
                time.sleep(0.5)
        raise TimeoutError(f"Could not connect to controller TCP server at {self.host}:{self.port}")

    def notify_read(self):
        while True:
            self.sock.sendall(b"read")
            response = self.sock.recv(8)
            if response == b"received":
                return
            time.sleep(0.5)

    def close(self):
        if self.sock is None:
            return
        try:
            self.sock.sendall(b"exit")
        except OSError:
            pass
        try:
            self.sock.close()
        except OSError:
            pass
        self.sock = None


def automatic_reading(cont_m_displacement, pr, destination_path):
    request_dir = Path(destination_path) / "request"
    if not request_dir.is_dir():
        raise FileNotFoundError(f"Request folder does not exist: {request_dir}")

    request_file = request_dir / f"{cont_m_displacement}.txt"
    while not request_file.exists():
        time.sleep(0.1)

    with request_file.open("r", encoding="utf-8") as file:
        k_points = int(_split_value(next(file)))
        m_displacement = int(_split_value(next(file)))
        po_size = int(_split_value(next(file)))

        next(file)
        next(file)

        pr.append(_split_pair(next(file)))

        next(file)
        next(file)

        final_objective = _split_pair(next(file))

        next(file)
        next(file)

        pdp = [_split_pair(next(file)) for _ in range(k_points)]

        next(file)
        next(file)

        po = [_split_pair(next(file)) for _ in range(po_size)]

    return k_points, m_displacement, po_size, pr, final_objective, pdp, po


def automatic_save(z, m_displacement, destination_path):
    response_dir = Path(destination_path) / "response"
    response_dir.mkdir(parents=True, exist_ok=True)
    response_file = response_dir / f"{m_displacement}.txt"

    with response_file.open("w", encoding="utf-8") as file:
        file.write("pOL(m)(metaheuristic result): X Y\n")
        file.write(f"{z[0]:f} {z[1]:f}")

    return m_displacement + 1


def save_log(destination_path, payload):
    log_path = Path(destination_path) / "log.txt"
    with log_path.open("w", encoding="utf-8") as file:
        file.write("Dados das Variaveis:\n\n")
        file.write("Tempo de CPU: \n")
        for value in payload["cpu_times"]:
            file.write(f" {value:12.8f} \n")
        file.write("\nSomatorio do tempo de CPU gasto: \n")
        file.write(f" {sum(payload['cpu_times']):6.8f} \n\n")
        file.write("Numero de deslocamentos: \n")
        file.write(f" {payload['displacements']} \n\n")
        file.write("Pontos do robo (centros): \n")
        for x, y in payload["route"]:
            file.write(f"{x:6.8f} {y:6.8f}\n")
        file.write("\n\n Distancia percorrida: \n")
        file.write(f" {payload['route_distance']:6.8f} \n\n")
        file.write("Somatorio do tempo de relogio gasto: \n")
        file.write(f" {sum(payload['wall_times']):6.8f} \n\n")
        file.write("Semente utilizada neste experimento: \n")
        file.write(f" {payload['seed']} \n")
        if payload.get("optimizer_info"):
            file.write("\n\n--- Informacoes do otimizador ---\n")
            for key, value in payload["optimizer_info"].items():
                file.write(f"{key}: {value}\n")


def save_plot_placeholders(destination_path, payload):
    destination = Path(destination_path)
    figure_payload = json.dumps(payload, indent=2)

    for filename in ("Data_and_displacement.fig", "Route.fig"):
        (destination / filename).write_text(figure_payload, encoding="utf-8")

    # Placeholder 1x1 JPEG so downstream paths keep existing names.
    jpeg_bytes = (
        b"\xff\xd8\xff\xe0\x00\x10JFIF\x00\x01\x01\x00\x00\x01\x00\x01\x00\x00"
        b"\xff\xdb\x00C\x00\x08\x06\x06\x07\x06\x05\x08\x07\x07\x07\x09\x09\x08"
        b"\x0a\x0c\x14\x0d\x0c\x0b\x0b\x0c\x19\x12\x13\x0f\x14\x1d\x1a\x1f\x1e"
        b"\x1d\x1a\x1c\x1c $.',#\x1c\x1c(7),01444\x1f'9=82<.342"
        b"\xff\xc0\x00\x11\x08\x00\x01\x00\x01\x03\x01\"\x00\x02\x11\x01\x03\x11\x01"
        b"\xff\xc4\x00\x14\x00\x01\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00"
        b"\x00\x00\x00\x08\xff\xc4\x00\x14\x10\x01\x00\x00\x00\x00\x00\x00\x00\x00"
        b"\x00\x00\x00\x00\x00\x00\x00\x08\xff\xda\x00\x08\x01\x01\x00\x00?\x00\xd2"
        b"\xcf \xff\xd9"
    )
    (destination / "figure.jpg").write_bytes(jpeg_bytes)


def _split_value(line):
    return line.split("/-/")[1].strip()


def _split_pair(line):
    left, right = [item.strip() for item in line.split("/-/")]
    return [float(left), float(right)]
