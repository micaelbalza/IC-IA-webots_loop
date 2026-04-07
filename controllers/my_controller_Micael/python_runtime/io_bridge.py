import json
import shutil
import socket
import subprocess
import tempfile
import time
from pathlib import Path


class ControlProgramClient:
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


def save_plot_artifacts(destination_path, payload):
    destination = Path(destination_path)
    figure_payload = json.dumps(payload, indent=2)

    for filename in ("Data_and_displacement.fig", "Route.fig"):
        (destination / filename).write_text(figure_payload, encoding="utf-8")

    points = _collect_points(payload)
    if not points:
        points = [[0.0, 0.0], [1.0, 1.0]]

    with tempfile.TemporaryDirectory() as temp_dir:
        temp_dir_path = Path(temp_dir)
        data_ppm = temp_dir_path / "data_and_displacement.ppm"
        route_ppm = temp_dir_path / "route.ppm"

        _write_plot_ppm(data_ppm, payload, mode="data")
        _write_plot_ppm(route_ppm, payload, mode="route")

        data_jpg = destination / "Data_and_displacement.jpg"
        route_jpg = destination / "Route.jpg"

        _convert_ppm_to_jpg(data_ppm, data_jpg)
        _convert_ppm_to_jpg(route_ppm, route_jpg)

        shutil.copyfile(data_jpg, destination / "figure.jpg")


def _split_value(line):
    return line.split("/-/")[1].strip()


def _split_pair(line):
    left, right = [item.strip() for item in line.split("/-/")]
    return [float(left), float(right)]


def _collect_points(payload):
    points = []

    for point in payload.get("route", []):
        points.append(point)

    for iteration in payload.get("iterations", []):
        points.extend(iteration.get("pdp_space", []))
        points.extend(iteration.get("obstacles", []))
        if iteration.get("current_position"):
            points.append(iteration["current_position"])
        if iteration.get("chosen_point"):
            points.append(iteration["chosen_point"])
        if iteration.get("final_objective"):
            points.append(iteration["final_objective"])

    return points


def _write_plot_ppm(ppm_path, payload, mode):
    width = 1200
    height = 900
    margin = 60

    image = [[[255, 255, 255] for _ in range(width)] for _ in range(height)]
    points = _collect_points(payload)
    min_x, max_x, min_y, max_y = _bounds(points)

    def to_pixel(point):
        x, y = point
        usable_width = width - 2 * margin
        usable_height = height - 2 * margin
        pixel_x = margin + int((x - min_x) / (max_x - min_x) * usable_width)
        pixel_y = height - margin - int((y - min_y) / (max_y - min_y) * usable_height)
        return pixel_x, pixel_y

    _draw_axes(image, width, height, margin)

    if mode == "data":
        for iteration in payload.get("iterations", []):
            pdp_space = iteration.get("pdp_space", [])
            obstacles = iteration.get("obstacles", [])
            current_position = iteration.get("current_position")
            chosen_point = iteration.get("chosen_point")
            final_objective = iteration.get("final_objective")

            _draw_polyline(image, pdp_space, to_pixel, [40, 90, 180], closed=False, thickness=2)
            _draw_points(image, obstacles, to_pixel, [180, 90, 20], radius=3)
            _draw_points(image, [final_objective] if final_objective else [], to_pixel, [20, 150, 20], radius=7)
            _draw_points(image, [current_position] if current_position else [], to_pixel, [80, 80, 80], radius=5)
            _draw_points(image, [chosen_point] if chosen_point else [], to_pixel, [200, 40, 40], radius=5)

        _draw_polyline(image, payload.get("route", []), to_pixel, [0, 0, 0], closed=False, thickness=3)
    else:
        route = payload.get("route", [])
        _draw_polyline(image, route, to_pixel, [0, 0, 0], closed=False, thickness=4)
        _draw_points(image, route, to_pixel, [200, 40, 40], radius=4)
        if route:
            _draw_points(image, [route[0]], to_pixel, [40, 90, 180], radius=6)
            _draw_points(image, [route[-1]], to_pixel, [20, 150, 20], radius=6)

    with ppm_path.open("w", encoding="ascii") as ppm:
        ppm.write(f"P3\n{width} {height}\n255\n")
        for row in image:
            ppm.write(" ".join(f"{r} {g} {b}" for r, g, b in row))
            ppm.write("\n")


def _bounds(points):
    xs = [point[0] for point in points]
    ys = [point[1] for point in points]
    min_x = min(xs)
    max_x = max(xs)
    min_y = min(ys)
    max_y = max(ys)

    if min_x == max_x:
        min_x -= 1.0
        max_x += 1.0
    if min_y == max_y:
        min_y -= 1.0
        max_y += 1.0

    padding_x = (max_x - min_x) * 0.1
    padding_y = (max_y - min_y) * 0.1
    return min_x - padding_x, max_x + padding_x, min_y - padding_y, max_y + padding_y


def _draw_axes(image, width, height, margin):
    for x in range(margin, width - margin):
        _set_pixel(image, x, height - margin, [220, 220, 220])
    for y in range(margin, height - margin):
        _set_pixel(image, margin, y, [220, 220, 220])


def _draw_polyline(image, points, to_pixel, color, closed, thickness):
    if len(points) < 2:
        return

    for index in range(len(points) - 1):
        _draw_line(image, to_pixel(points[index]), to_pixel(points[index + 1]), color, thickness)

    if closed:
        _draw_line(image, to_pixel(points[-1]), to_pixel(points[0]), color, thickness)


def _draw_points(image, points, to_pixel, color, radius):
    for point in points:
        _draw_circle(image, to_pixel(point), radius, color)


def _draw_line(image, start, end, color, thickness):
    x0, y0 = start
    x1, y1 = end
    dx = abs(x1 - x0)
    dy = -abs(y1 - y0)
    sx = 1 if x0 < x1 else -1
    sy = 1 if y0 < y1 else -1
    error = dx + dy

    while True:
        _draw_circle(image, (x0, y0), thickness, color)
        if x0 == x1 and y0 == y1:
            break
        error2 = 2 * error
        if error2 >= dy:
            error += dy
            x0 += sx
        if error2 <= dx:
            error += dx
            y0 += sy


def _draw_circle(image, center, radius, color):
    cx, cy = center
    for dy in range(-radius, radius + 1):
        for dx in range(-radius, radius + 1):
            if dx * dx + dy * dy <= radius * radius:
                _set_pixel(image, cx + dx, cy + dy, color)


def _set_pixel(image, x, y, color):
    if 0 <= y < len(image) and 0 <= x < len(image[0]):
        image[y][x] = color


def _convert_ppm_to_jpg(source_path, destination_path):
    subprocess.run(
        [
            "ffmpeg",
            "-y",
            "-loglevel",
            "error",
            "-i",
            str(source_path),
            str(destination_path),
        ],
        check=True,
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL,
    )
