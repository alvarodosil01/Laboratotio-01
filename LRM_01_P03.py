"""Parte 03: programa autonomo del Grupo 4.
Percepcion, potenciales, control, supervisor y registro separados internamente.
Implementacion pendiente de calibracion y validacion fisica, sin simulaciones.
"""
import argparse
import asyncio
import csv
import json
import math
import time
from collections import deque
from dataclasses import dataclass, asdict
from datetime import datetime
from pathlib import Path

PARTE = 3

@dataclass(frozen=True)
class Configuracion:
    robot: str = "C3_UIEC_Grupo4"
    meta_x: float = 0.0
    meta_y: float = 1.0
    meta_theta: float | None = None
    potencial: str = "hibrido"
    ganancia: float = 1.0
    transicion: float = 0.30
    kv: float = 0.4
    kw: float = 1.5
    vmax: float = 0.05
    wmax: float = 0.8
    rueda_max: float = 0.10
    separacion_ruedas: float = 0.235  # Heredada de T03: medir antes de validar.
    tolerancia: float = 0.05
    tolerancia_angular: float = math.radians(3)
    periodo: float = 0.10
    tiempo_max: float = 120.0
    timeout_lectura: float = 2.0

    def validar(self):
        for nombre, valor in asdict(self).items():
            if isinstance(valor, (int, float)) and not math.isfinite(valor):
                raise ValueError(f"{nombre}: debe ser finito")
        for nombre in ("ganancia", "transicion", "kv", "kw", "vmax", "wmax",
                       "rueda_max", "separacion_ruedas", "tolerancia",
                       "tolerancia_angular", "periodo", "tiempo_max", "timeout_lectura"):
            if getattr(self, nombre) <= 0:
                raise ValueError(f"{nombre}: debe ser mayor que cero")
        if self.potencial not in ("cuadratico", "conico", "hibrido", "suave"):
            raise ValueError("Potencial desconocido")
        if not self.robot.strip():
            raise ValueError("Falta el nombre Bluetooth")
        return self


@dataclass(frozen=True)
class Pose:
    x: float
    y: float
    theta: float


def angulo_corto(a):
    return math.atan2(math.sin(a), math.cos(a))


def velocidades(fx, fy, distancia, config):
    """Avance y giro simultaneos. Reducir avance si el objetivo queda de lado.

    Si queda detras, v=0 y el mismo controlador sigue calculando w; no se
    ejecuta una maniobra turn_left seguida de move. El limite por distancia
    evita atravesar la meta con el potencial conico de fuerza constante.
    """
    error = math.atan2(fy, fx)
    w = max(-config.wmax, min(config.wmax, config.kw*error))
    v = min(config.vmax, config.kv*math.hypot(fx, fy), config.kv*distancia)
    return v*max(0.0, math.cos(error)), w


def ruedas(v, w, config):
    """Velocidades tangenciales (m/s), no angulares: no se usa radio de rueda.

    Si una rueda supera el limite se escalan ambas por el mismo factor para
    conservar la curvatura. Devuelve tambien las velocidades efectivas v,w.
    """
    izq, der = v-w*config.separacion_ruedas/2, v+w*config.separacion_ruedas/2
    escala = max(1.0, abs(izq)/config.rueda_max, abs(der)/config.rueda_max)
    return izq/escala, der/escala, v/escala, w/escala


def atractivo(dx, dy, config):
    """Devuelve (U, Fx, Fy) globales. dx,dy apuntan desde el robot a la meta.

    La propuesta suave usa U=k*d0*(sqrt(d^2+d0^2)-d0). Su fuerza se
    satura suavemente lejos de la meta y tiende a cero al llegar.
    """
    d = math.hypot(dx, dy)
    if d == 0:
        return 0.0, 0.0, 0.0
    k, d0 = config.ganancia, config.transicion
    if config.potencial == "cuadratico":
        u, magnitud = 0.5 * k * d*d, k*d
    elif config.potencial == "conico":
        u, magnitud = k*d, k  # No diferenciable en cero: caso tratado arriba.
    elif config.potencial == "hibrido":
        u = 0.5*k*d*d if d <= d0 else k*d0*(d - 0.5*d0)
        magnitud = k * min(d, d0)
    elif config.potencial == "suave":
        raiz = math.hypot(d, d0)
        u, magnitud = k*d0*(raiz-d0), k*d0*d/raiz
    else:
        raise ValueError("Potencial no implementado")
    return u, magnitud*dx/d, magnitud*dy/d


def a_local(fx, fy, theta):
    """R(theta)^T: eje X local hacia delante, Y local hacia la izquierda."""
    c, s = math.cos(theta), math.sin(theta)
    return c*fx+s*fy, -s*fx+c*fy


# CONFIGURACION DE LA NAVEGACION CON OBSTACULOS
@dataclass(frozen=True)
class ConfiguracionNavegacion(Configuracion):
    angulos_ir: tuple = ()  # Radianes, en el orden REAL del vector del SDK.
    repulsivo: str = "khatib"
    krep: float = 0.002
    influencia: float = 0.25
    fuerza_max_sensor: float = 2.0
    filtro_alpha: float = 0.5
    seguridad: float = 0.06
    ventana: float = 4.0
    progreso_min: float = 0.02
    movimiento_min: float = 0.03
    fuerza_min: float = 0.02
    escape: str = "tangencial"
    duracion_escape: float = 8.0
    ganancia_escape: float = 0.4
    distancia_submeta: float = 0.35
    max_escapes: int = 4

    def validar(self):
        super().validar()
        if len(self.angulos_ir) != 7 or not all(math.isfinite(x) for x in self.angulos_ir):
            raise ValueError("Introduzca los 7 angulos IR verificados, en orden del SDK")
        if any(abs(a) > math.pi/2+1e-6 for a in self.angulos_ir):
            raise ValueError("Los sensores frontales deben estar entre -90 y +90 grados")
        if len(set(self.angulos_ir)) != 7:
            raise ValueError("Cada sensor debe tener un angulo distinto")
        if self.repulsivo not in ("khatib", "polinomico"):
            raise ValueError("Potencial repulsivo desconocido")
        if self.escape not in ("ninguno", "tangencial", "submeta"):
            raise ValueError("Estrategia de escape desconocida")
        for nombre in ("krep", "influencia", "fuerza_max_sensor", "seguridad", "ventana",
                       "progreso_min", "movimiento_min", "fuerza_min", "duracion_escape",
                       "ganancia_escape", "distancia_submeta"):
            if getattr(self, nombre) <= 0:
                raise ValueError(f"{nombre} debe ser positivo")
        if not 0 < self.filtro_alpha <= 1:
            raise ValueError("alpha debe estar entre 0 (excluido) y 1")
        if not 0.05 < self.seguridad < self.influencia <= 0.25:
            raise ValueError("La tabla heredada exige 5 cm < seguridad < influencia <= 25 cm")
        if self.ventana <= self.periodo or self.duracion_escape <= self.periodo:
            raise ValueError("Ventana y escape deben superar el periodo de control")
        if self.max_escapes < 1:
            raise ValueError("Debe permitirse al menos un intento de escape")
        return self


def configurar():
    p = argparse.ArgumentParser(description=f"Parte {PARTE:02}: campos de potencial y control continuo")
    p.add_argument("--robot", default="C3_UIEC_Grupo4")
    p.add_argument("--x-cm", type=float)
    p.add_argument("--y-cm", type=float)
    p.add_argument("--theta-grados", type=float)
    p.add_argument("--potencial", choices=["cuadratico", "conico", "hibrido", "suave"], default="hibrido")
    p.add_argument("--ganancia", type=float, default=1.0)
    p.add_argument("--transicion-cm", type=float, default=30)
    p.add_argument("--kv", type=float, default=0.4)
    p.add_argument("--kw", type=float, default=1.5)
    p.add_argument("--vmax-cm-s", type=float, default=5)
    p.add_argument("--wmax-grados-s", type=float, default=math.degrees(0.8))
    p.add_argument("--rueda-max-cm-s", type=float, default=10)
    p.add_argument("--separacion-cm", type=float, default=23.5)
    p.add_argument("--tolerancia-cm", type=float, default=5)
    p.add_argument("--periodo", type=float, default=0.1)
    p.add_argument("--tiempo-max", type=float, default=120)
    p.add_argument("--angulos-grados", type=float, nargs=7,
                   help="7 angulos medidos en orden SDK: 0 delante, positivos a izquierda")
    p.add_argument("--repulsivo", choices=["khatib", "polinomico"], default="khatib")
    p.add_argument("--krep", type=float, default=0.002)
    p.add_argument("--influencia-cm", type=float, default=25)
    p.add_argument("--fuerza-max-sensor", type=float, default=2)
    p.add_argument("--alpha", type=float, default=0.5)
    p.add_argument("--seguridad-cm", type=float, default=6)
    p.add_argument("--calibracion-json", type=Path,
                   help="Tabla opcional {distancia_cm: [s1,...,s7]}, entre 5 y 25 cm")
    if PARTE == 3:
        p.add_argument("--escape", choices=["ninguno", "tangencial", "submeta"], default="tangencial")
        p.add_argument("--ventana", type=float, default=4)
        p.add_argument("--progreso-cm", type=float, default=2)
        p.add_argument("--movimiento-cm", type=float, default=3)
        p.add_argument("--fuerza-min", type=float, default=0.02)
        p.add_argument("--duracion-escape", type=float, default=8)
        p.add_argument("--ganancia-escape", type=float, default=0.4)
        p.add_argument("--submeta-cm", type=float, default=35)
        p.add_argument("--max-escapes", type=int, default=4)
    a = p.parse_args()
    try:
        x = a.x_cm if a.x_cm is not None else float(input("Meta X (cm): "))
        y = a.y_cm if a.y_cm is not None else float(input("Meta Y (cm): "))
        angulos = a.angulos_grados
        if angulos is None:
            angulos = [float(v) for v in input("7 angulos IR verificados, separados por espacios (grados): ").split()]
        c = ConfiguracionNavegacion(robot=a.robot, meta_x=x/100, meta_y=y/100,
            meta_theta=None if a.theta_grados is None else math.radians(a.theta_grados),
            potencial=a.potencial, ganancia=a.ganancia, transicion=a.transicion_cm/100,
            kv=a.kv, kw=a.kw, vmax=a.vmax_cm_s/100, wmax=math.radians(a.wmax_grados_s),
            rueda_max=a.rueda_max_cm_s/100, separacion_ruedas=a.separacion_cm/100,
            tolerancia=a.tolerancia_cm/100, periodo=a.periodo, tiempo_max=a.tiempo_max,
            angulos_ir=tuple(math.radians(v) for v in angulos), repulsivo=a.repulsivo,
            krep=a.krep, influencia=a.influencia_cm/100, fuerza_max_sensor=a.fuerza_max_sensor,
            filtro_alpha=a.alpha, seguridad=a.seguridad_cm/100,
            escape=getattr(a, "escape", "ninguno"), ventana=getattr(a, "ventana", 4),
            progreso_min=getattr(a, "progreso_cm", 2)/100,
            movimiento_min=getattr(a, "movimiento_cm", 3)/100,
            fuerza_min=getattr(a, "fuerza_min", 0.02),
            duracion_escape=getattr(a, "duracion_escape", 8),
            ganancia_escape=getattr(a, "ganancia_escape", 0.4),
            distancia_submeta=getattr(a, "submeta_cm", 35)/100,
            max_escapes=getattr(a, "max_escapes", 4)).validar()
        tabla = MUESTRAS_IR
        if a.calibracion_json:
            tabla = {float(k): v for k, v in json.loads(a.calibracion_json.read_text(encoding="utf-8")).items()}
        calibrador = CalibradorIR(tabla)
    except (ValueError, TypeError, EOFError, OSError) as exc:
        p.error(str(exc))
    print(f"P{PARTE:02} | meta ({x}, {y}) cm | {c.potencial} + {c.repulsivo} | escape: {c.escape}")
    print("Origen fuera de la base. Rumbo inicial SDK: 90 grados, +Y hacia delante.")
    print("Calibracion y ganancias iniciales: comprobar en laboratorio. No hay sensores traseros.")
    input("Intro para conectar; Ctrl+C cancela: ")
    return c, calibrador


# PERCEPCION: MUESTRAS DEL EQUIPO DE T03/SESION 04, NO UNA CALIBRACION UNIVERSAL.
MUESTRAS_IR = {
    5: [1377, 1350, 902.7, 1517, 1111, 1031.5, 1103],
    7.5: [512, 674, 415, 680, 460.2, 363, 461.1],
    10: [251, 356, 264, 361.1, 221.5, 153.5, 261.9],
    12.5: [155, 200, 172.4, 217.4, 132.6, 73.6, 150],
    15: [94.5, 126, 121.5, 131.7, 96.5, 36.6, 95.8],
    20: [42, 67.3, 68.8, 72.7, 55.5, 15.6, 50.2],
    25: [27, 41.4, 41.2, 42.3, 40.6, 15.1, 31.8],
}


class CalibradorIR:
    def __init__(self, muestras):
        self.muestras = dict(sorted(muestras.items()))
        if not self.muestras or min(self.muestras) != 5 or max(self.muestras) != 25:
            raise ValueError("Se requieren muestras desde 5 hasta 25 cm")
        if any(not math.isfinite(d) or d <= 0 for d in self.muestras):
            raise ValueError("Distancias de calibracion invalidas")
        for fila in self.muestras.values():
            if len(fila) != 7 or any(not math.isfinite(v) or v <= 0 for v in fila):
                raise ValueError("Cada distancia requiere 7 intensidades positivas y finitas")
        self.tablas = [tuple((fila[i], d/100) for d, fila in self.muestras.items()) for i in range(7)]
        if any(any(a[0] <= b[0] for a, b in zip(t, t[1:])) for t in self.tablas):
            raise ValueError("La intensidad de cada sensor debe disminuir con la distancia")

    def convertir(self, valores):
        if len(valores) != 7 or any(v is None or not math.isfinite(v) or v < 0 for v in valores):
            raise RuntimeError("Lectura IR invalida; se detiene la mision")
        return tuple(self.distancia(v, t) for v, t in zip(valores, self.tablas))

    @staticmethod
    def distancia(valor, tabla):
        if valor < tabla[-1][0]:
            return math.inf  # Sin deteccion util, no una medida de 50 cm.
        if valor >= tabla[0][0]:
            return tabla[0][1]  # Saturacion cercana; activa la parada de seguridad.
        for (a, da), (b, db) in zip(tabla, tabla[1:]):
            if b <= valor <= a:
                return da+(a-valor)/(a-b)*(db-da)
        raise RuntimeError("No se pudo interpolar la lectura IR")


class FiltroIR:
    def __init__(self, alpha):
        self.alpha, self.anteriores = alpha, None

    def actualizar(self, distancias):
        # Una deteccion nueva entra de inmediato. Una desaparicion sale de
        # inmediato; nunca se hace aritmetica entre infinito y una distancia.
        if self.anteriores is None:
            self.anteriores = distancias
        self.anteriores = tuple(
            d if not math.isfinite(d) or not math.isfinite(prev)
            else self.alpha*d+(1-self.alpha)*prev
            for d, prev in zip(distancias, self.anteriores))
        return self.anteriores


# REPULSION: TODOS LOS VECTORES SE EXPRESAN EN EL MARCO LOCAL DEL ROBOT.
def repulsion(distancias, config):
    """Khatib: U=k/2*(1/d-1/d0)^2; alternativa U=k/2*(1-d/d0)^2.

    Ambas son cero fuera de d0. Fuerza = -grad(U), hacia fuera del obstaculo.
    La magnitud de cada fuerza se limita como regularizacion del controlador.
    Sus ganancias tienen distintas unidades: ajustar por separado.
    """
    fx = fy = potencial = 0.0
    for distancia, angulo in zip(distancias, config.angulos_ir):
        if not math.isfinite(distancia) or distancia >= config.influencia:
            continue
        d, d0, k = max(distancia, 0.005), config.influencia, config.krep
        if config.repulsivo == "khatib":
            potencial += 0.5*k*(1/d-1/d0)**2
            magnitud = k*(1/d-1/d0)/d**2
        else:
            potencial += 0.5*k*(1-d/d0)**2
            magnitud = k*(1-d/d0)/d0
        magnitud = min(magnitud, config.fuerza_max_sensor)
        fx -= magnitud*math.cos(angulo)
        fy -= magnitud*math.sin(angulo)
    return potencial, fx, fy


@dataclass(frozen=True)
class Lectura:
    pose: Pose
    intensidades: tuple
    distancias: tuple
    filtradas: tuple


@dataclass(frozen=True)
class Decision:
    estado: str
    distancia: float
    u_atr: float
    u_rep: float
    f_atr: tuple
    f_rep: tuple
    f_extra: tuple
    f_res: tuple
    comando: tuple
    evento: str = ""


class SupervisorPasivo:
    """P02 no oculta los minimos locales: sirve como referencia sin escape."""
    escapes = 0

    def corregir(self, pose, t, distancia, fuerza, repulsiva, distancias, config):
        return fuerza, "NAVEGACION", ""


def decidir(lectura, config, supervisor, t):
    pose = lectura.pose
    dx, dy = config.meta_x-pose.x, config.meta_y-pose.y
    distancia = math.hypot(dx, dy)
    ua, fx, fy = atractivo(dx, dy, config)
    atr = a_local(fx, fy, pose.theta)
    ur, rx, ry = repulsion(lectura.filtradas, config)
    rep = (rx, ry)
    res = (atr[0]+rx, atr[1]+ry)
    # La seguridad usa medidas SIN filtrar: el filtro no puede retrasar la parada.
    if min(lectura.distancias) <= config.seguridad:
        return Decision("EMERGENCIA", distancia, ua, ur, atr, rep, (0, 0), res,
                        (0, 0, 0, 0), "Distancia IR critica")
    if distancia <= config.tolerancia:
        error = 0 if config.meta_theta is None else angulo_corto(config.meta_theta-pose.theta)
        estado = "LLEGADA" if abs(error) <= config.tolerancia_angular else "ORIENTACION_FINAL"
        w = 0 if estado == "LLEGADA" else max(-config.wmax, min(config.wmax, config.kw*error))
        return Decision(estado, distancia, ua, ur, atr, rep, (0, 0), res, ruedas(0, w, config))
    corregida, estado, evento = supervisor.corregir(pose, t, distancia, res, rep, lectura.filtradas, config)
    extra = (corregida[0]-res[0], corregida[1]-res[1])
    v, w = velocidades(*corregida, distancia, config)
    return Decision(estado, distancia, ua, ur, atr, rep, extra, corregida, ruedas(v, w, config), evento)


# SUPERVISOR P03: ESTADOS EXPLICITOS Y DOS ESTRATEGIAS DE ESCAPE.
class DetectorMinimo:
    """Falta de progreso lejos de meta, con poco desplazamiento o fuerza baja.

    La ventana temporal distingue un atasco de una lectura aislada. La distancia
    recorrida neta detecta tambien oscilaciones que vuelven al mismo punto.
    No es una demostracion matematica de minimo: es un detector experimental.
    """
    def __init__(self):
        self.historial = deque()

    def reiniciar(self):
        self.historial.clear()

    def actualizar(self, pose, t, distancia, fuerza, config):
        self.historial.append((t, pose.x, pose.y, distancia))
        while len(self.historial) > 1 and self.historial[1][0] <= t-config.ventana:
            self.historial.popleft()
        inicio = self.historial[0]
        if t-inicio[0] < config.ventana or distancia <= config.tolerancia:
            return False
        progreso = inicio[3]-distancia
        desplazamiento = math.hypot(pose.x-inicio[1], pose.y-inicio[2])
        return progreso < config.progreso_min and (
            desplazamiento < config.movimiento_min or math.hypot(*fuerza) < config.fuerza_min)


class Supervisor:
    def __init__(self):
        self.detector = DetectorMinimo()
        self.estado = "NAVEGACION"
        self.escapes = 0
        self.inicio_escape = 0.0
        self.distancia_inicial = 0.0
        self.lado = 1
        self.submeta = None

    @staticmethod
    def lado_libre(distancias, config):
        # Indices dados por el orden calibrado, no por una numeracion supuesta.
        izquierda = [min(d, config.influencia) for d, a in zip(distancias, config.angulos_ir) if a > 0]
        derecha = [min(d, config.influencia) for d, a in zip(distancias, config.angulos_ir) if a < 0]
        li = sum(izquierda)/len(izquierda) if izquierda else 0
        ld = sum(derecha)/len(derecha) if derecha else 0
        return 1 if li >= ld else -1

    def iniciar_escape(self, pose, t, distancia, distancias, config):
        if self.escapes >= config.max_escapes:
            self.estado = "EMERGENCIA"
            raise RuntimeError("Limite de escapes alcanzado sin completar la mision")
        preferido = self.lado_libre(distancias, config)
        # Si el mismo lado no resolvio el problema, alternar en el siguiente intento.
        self.lado = preferido if self.escapes == 0 else -self.lado
        self.escapes += 1
        self.inicio_escape, self.distancia_inicial = t, distancia
        self.estado = "ESCAPE_TANGENCIAL" if config.escape == "tangencial" else "ESCAPE_SUBMETA"
        angulo = pose.theta+self.lado*math.pi/2
        self.submeta = (pose.x+config.distancia_submeta*math.cos(angulo),
                        pose.y+config.distancia_submeta*math.sin(angulo))
        self.detector.reiniciar()

    def corregir(self, pose, t, distancia, fuerza, repulsiva, distancias, config):
        evento = ""
        if self.estado == "NAVEGACION":
            if not self.detector.actualizar(pose, t, distancia, fuerza, config):
                return fuerza, self.estado, evento
            if config.escape == "ninguno":
                self.detector.reiniciar()
                return fuerza, "MINIMO_LOCAL", "Falta de progreso detectada; recuperacion desactivada"
            self.iniciar_escape(pose, t, distancia, distancias, config)
            evento = f"Inicio de escape {self.escapes}: {config.escape}"

        transcurrido = t-self.inicio_escape
        mejora = distancia <= self.distancia_inicial-config.progreso_min
        submeta_alcanzada = config.escape == "submeta" and math.hypot(
            pose.x-self.submeta[0], pose.y-self.submeta[1]) <= config.tolerancia
        if transcurrido >= config.duracion_escape or submeta_alcanzada or (transcurrido >= 1 and mejora):
            self.estado = "NAVEGACION"
            self.detector.reiniciar()
            # Finalizar el intento no significa que la mision haya tenido exito.
            return fuerza, self.estado, "Fin de intento; se reanuda la meta original"

        if config.escape == "tangencial":
            rx, ry = repulsiva
            norma = math.hypot(rx, ry)
            tx, ty = (ry/norma, -rx/norma) if norma > 1e-9 else (0, 1)
            # Mantener la repulsion y anadir una componente lateral de lado fijo.
            fuerza = (fuerza[0]+self.lado*config.ganancia_escape*tx,
                      fuerza[1]+self.lado*config.ganancia_escape*ty)
        else:
            # Meta temporal global fija al comenzar el escape; la original no cambia.
            _, fx, fy = atractivo(self.submeta[0]-pose.x, self.submeta[1]-pose.y, config)
            local = a_local(fx, fy, pose.theta)
            fuerza = (local[0]+repulsiva[0], local[1]+repulsiva[1])
        return fuerza, self.estado, evento


def crear_supervisor(config):
    return Supervisor()


# REGISTRO: MEDIDAS SIN FILTRAR, FILTRADAS Y FUERZAS POR SEPARADO.
class Registro:
    CAMPOS = ("t_s", "Ts_s", "dt_s", "x_m", "y_m", "theta_rad", "distancia_meta_m",
              "U_atr", "U_rep", "F_atr_x", "F_atr_y", "F_rep_x", "F_rep_y",
              "F_extra_x", "F_extra_y", "F_res_x", "F_res_y", "v_m_s", "w_rad_s",
              "v_izq_m_s", "v_der_m_s", "estado", "evento", "retraso_s") + tuple(
                  f"{prefijo}{i}" for prefijo in ("ir_", "d_m_", "d_filtrada_m_") for i in range(1, 8))

    def __init__(self, config, calibrador):
        self.carpeta = Path(__file__).resolve().parent / "datos" / Path(__file__).stem / datetime.now().strftime("%Y%m%d_%H%M%S_%f")
        self.carpeta.mkdir(parents=True)
        for nombre, datos in (("configuracion", asdict(config)), ("calibracion", calibrador.muestras)):
            (self.carpeta/f"{nombre}.json").write_text(json.dumps(datos, indent=2), encoding="utf-8")
        self.archivo = (self.carpeta/"recorrido.csv").open("w", newline="", encoding="utf-8")
        self.csv = csv.DictWriter(self.archivo, fieldnames=self.CAMPOS)
        self.csv.writeheader()
        self.minima, self.vmax, self.retrasos, self.colisiones = math.inf, 0.0, 0, 0

    def ciclo(self, t, dt, lectura, decision, retraso, config):
        d, p = decision, lectura.pose
        izq, der, v, w = d.comando
        self.minima = min(self.minima, *lectura.distancias)
        self.vmax = max(self.vmax, abs(v))
        self.retrasos += int(dt > config.periodo*1.5)
        valores = (t, config.periodo, dt, p.x, p.y, p.theta, d.distancia, d.u_atr, d.u_rep,
                   *d.f_atr, *d.f_rep, *d.f_extra, *d.f_res, v, w, izq, der,
                   d.estado, d.evento, retraso, *lectura.intensidades,
                   *(v if math.isfinite(v) else "" for v in lectura.distancias),
                   *(v if math.isfinite(v) else "" for v in lectura.filtradas))
        self.csv.writerow(dict(zip(self.CAMPOS, valores)))
        self.archivo.flush()

    def evento(self, t, texto):
        self.csv.writerow({"t_s": t, "estado": "FIN", "evento": texto})
        self.archivo.flush()

    def cerrar(self, resultado, tiempo, error, escapes, colision):
        self.archivo.close()
        datos = {"resultado": resultado, "exito": resultado == "meta_alcanzada", "tiempo_s": tiempo,
                 "error_final_m": error, "escapes": escapes, "colision": colision,
                 "distancia_minima_ir_m": self.minima if math.isfinite(self.minima) else None,
                 "vmax_ordenada_m_s": self.vmax, "ciclos_lentos": self.retrasos,
                 "nota": "Distancia minima estimada por IR, no separacion geometrica de todo el chasis."}
        (self.carpeta/"resultado.json").write_text(json.dumps(datos, indent=2), encoding="utf-8")


async def navegar(adaptador, config, calibrador):
    registro = Registro(config, calibrador)
    supervisor = crear_supervisor(config)
    inicio = anterior = plazo = time.monotonic()
    resultado, error = "interrumpida", None
    print(f"Registro: {registro.carpeta}")
    try:
        while True:
            adaptador.comprobar_seguridad()
            ahora = time.monotonic()
            if ahora-inicio >= config.tiempo_max:
                raise TimeoutError("Tiempo maximo de mision alcanzado")
            lectura = await adaptador.leer_entorno()
            decision = decidir(lectura, config, supervisor, ahora-inicio)
            error = decision.distancia
            await adaptador.aplicar(*decision.comando[:2])
            registro.ciclo(ahora-inicio, ahora-anterior, lectura, decision, max(0, ahora-plazo), config)
            if decision.evento:
                print(decision.estado, decision.evento)
            if decision.estado in ("LLEGADA", "EMERGENCIA"):
                resultado = "meta_alcanzada" if decision.estado == "LLEGADA" else decision.evento
                break
            anterior = ahora
            plazo += config.periodo
            actual = time.monotonic()
            if plazo < actual:
                plazo += (math.floor((actual-plazo)/config.periodo)+1)*config.periodo
            await asyncio.sleep(max(0, plazo-time.monotonic()))
    except BaseException as exc:
        resultado = f"{type(exc).__name__}: {exc}"
        raise
    finally:
        try:
            await adaptador.parar()
        finally:
            try:
                registro.evento(time.monotonic()-inicio, resultado)
            finally:
                registro.cerrar(resultado, time.monotonic()-inicio, error, supervisor.escapes, adaptador.colision)
    return resultado


# ADAPTADOR: SOLO ESTA CLASE Y ejecutar() CONOCEN EL SDK.
class Adaptador:
    def __init__(self, robot, config, calibrador):
        self.robot, self.config, self.calibrador = robot, config, calibrador
        self.filtro = FiltroIR(config.filtro_alpha)
        self.colision = False

    def comprobar_seguridad(self):
        if self.colision:
            raise RuntimeError("Parachoques activado; mision cancelada")

    async def leer_entorno(self):
        self.comprobar_seguridad()
        # Consultas secuenciales: no se presupone que el SDK serialice peticiones concurrentes.
        p = await asyncio.wait_for(self.robot.get_position(), self.config.timeout_lectura)
        if p is None or not all(math.isfinite(v) for v in (p.x, p.y, p.heading)):
            raise RuntimeError("Odometria no disponible")
        pose = Pose(p.x/100, p.y/100, math.radians(p.heading))
        ir = await asyncio.wait_for(self.robot.get_ir_proximity(), self.config.timeout_lectura)
        if ir is None:
            raise RuntimeError("Sensores IR no disponibles")
        valores = tuple(ir.sensors)
        distancias = self.calibrador.convertir(valores)
        self.comprobar_seguridad()
        return Lectura(pose, valores, distancias, self.filtro.actualizar(distancias))

    async def aplicar(self, izq, der):
        self.comprobar_seguridad()
        await asyncio.wait_for(self.robot.set_wheel_speeds(100*izq, 100*der), self.config.timeout_lectura)
        if self.colision:
            await self.parar()
            self.comprobar_seguridad()

    async def parar(self):
        await asyncio.wait_for(self.robot.set_wheel_speeds(0, 0), self.config.timeout_lectura)


def ejecutar(config, calibrador):
    from irobot_edu_sdk.backend.bluetooth import Bluetooth
    from irobot_edu_sdk.robots import Create3, event
    config.validar()
    robot = Create3(Bluetooth(config.robot))
    adaptador = Adaptador(robot, config, calibrador)

    @event(robot.when_bumped, [])
    async def al_chocar(r):
        adaptador.colision = True
        await adaptador.parar()
        await r.set_lights_blink_rgb(255, 0, 0)

    @event(robot.when_play)
    async def iniciar(r):
        try:
            await adaptador.parar()
            await asyncio.wait_for(r.reset_navigation(), config.timeout_lectura)
            await asyncio.sleep(0.2)
            await r.set_lights_on_rgb(0, 255, 0)
            await adaptador.leer_entorno()
            await r.set_lights_on_rgb(0, 0, 255)
            resultado = await navegar(adaptador, config, calibrador)
            print(resultado)
            await r.set_lights_on_rgb(*( (255, 80, 0) if resultado == "meta_alcanzada" else (255, 0, 0)))
        except Exception as exc:
            print(f"Mision detenida: {exc}")
        finally:
            try:
                await adaptador.parar()
            finally:
                try:
                    await asyncio.wait_for(r.stop(), config.timeout_lectura)
                finally:
                    await asyncio.wait_for(r.disconnect(), config.timeout_lectura)
    robot.play()


def main():
    ejecutar(*configurar())


if __name__ == "__main__":
    main()
