"""Parte 01: atraccion. Grupo 4.
Programa autonomo organizado por responsabilidades.
"""

import argparse
import asyncio
import csv
import json
import math
import time
from dataclasses import dataclass, asdict, replace
from datetime import datetime
from pathlib import Path


# CONFIGURACION
@dataclass(frozen=True)
class Configuracion:
    robot: str = "C3_UIEC_Grupo4"

    # Coordenadas respecto a los ejes fijos del escenario.
    meta_x: float = 0.0
    meta_y: float = 1.0
    meta_theta: float | None = None
    orientacion_inicial: float = math.pi / 2

    potencial: str = "hibrido"
    ganancia: float = 1.0
    transicion: float = 0.30
    kv: float = 2.0
    kw: float = 1.5

    # Unidades internas: metros, segundos y radianes.
    vmax: float = 0.50  # Limite del SDK: 500 mm/s = 50 cm/s.
    wmax: float = 0.8
    rueda_max: float = 0.50
    aceleracion: float = 0.10  # m/s2.
    frenado: float = 0.20  # m/s2: parametro del perfil de aproximacion.

    separacion_ruedas: float = 0.235  # Medir antes de validar.
    tolerancia: float = 0.05
    tolerancia_angular: float = math.radians(3)
    periodo: float = 0.10
    tiempo_max: float = 120.0
    timeout_lectura: float = 2.0

    # Se seleccionan y registran, pero no actuan en P01:
    # esta parte utiliza un escenario libre de obstaculos.
    repulsivo: str = "cuadratico"
    ganancia_repulsiva: float = 0.002
    influencia: float = 0.25

    def validar(self):
        for nombre, valor in asdict(self).items():
            if isinstance(valor, (int, float)) and not math.isfinite(valor):
                raise ValueError(f"{nombre}: debe ser finito")

        for nombre in (
            "ganancia", "transicion", "kv", "kw", "vmax", "wmax",
            "rueda_max", "separacion_ruedas", "tolerancia",
            "tolerancia_angular", "periodo", "tiempo_max",
            "timeout_lectura", "ganancia_repulsiva", "influencia",
            "aceleracion", "frenado"
        ):
            if getattr(self, nombre) <= 0:
                raise ValueError(f"{nombre}: debe ser mayor que cero")

        if self.potencial not in (
            "cuadratico", "conico", "hibrido", "suave"
        ):
            raise ValueError("Potencial desconocido")

        if not self.robot.strip():
            raise ValueError("Falta el nombre Bluetooth")

        if self.vmax > 0.50 or self.rueda_max > 0.50:
            raise ValueError(
                "El SDK limita el avance y cada rueda a 50 cm/s"
            )

        if self.repulsivo not in (
            "cuadratico", "inverso", "lineal_tramos", "propio"
        ):
            raise ValueError("Seleccion repulsiva desconocida")

        return self


def pedir_numero(texto, actual, positivo=False):
    """Intro conserva el valor. Una entrada incorrecta se vuelve a pedir."""
    while True:
        entrada = input(
            f"{texto} [{actual:g}]: "
        ).strip().replace(",", ".")

        try:
            valor = actual if not entrada else float(entrada)

            if not math.isfinite(valor) or (positivo and valor <= 0):
                raise ValueError

            return valor

        except ValueError:
            print(
                "Introduzca un numero finito"
                + (" mayor que cero." if positivo else ".")
            )


def pedir_orientacion_inicial():
    """Exigir un angulo escrito; no suponer que el robot mira hacia +Y."""
    print(
        "Ejes fijos del suelo: "
        "0 grados = +X; 90 = +Y; 180 = -X; -90 = -Y."
    )
    print(
        "El robot empieza en (0, 0), "
        "pero puede mirar en cualquier direccion."
    )

    while True:
        entrada = input(
            "Escriba la orientacion INICIAL del robot (grados): "
        ).strip()

        try:
            valor = float(entrada.replace(",", "."))

            if not math.isfinite(valor):
                raise ValueError

            return math.radians(valor)

        except ValueError:
            print(
                "Es obligatorio escribir un angulo finito, "
                "por ejemplo 45 o -90."
            )


def mostrar_configuracion(c):
    print("\n--- P01: configuracion de la mision ---")
    print(
        "Origen: (0, 0) inicial. "
        "X e Y son ejes fijos marcados en el suelo."
    )
    print(
        f"   Orientacion INICIAL: "
        f"{math.degrees(c.orientacion_inicial):g} grados"
    )

    print(f"1. Meta: X={c.meta_x*100:g} cm, Y={c.meta_y*100:g} cm")

    orientacion = (
        "sin orientacion final"
        if c.meta_theta is None
        else f"{math.degrees(c.meta_theta):g} grados"
    )

    print(
        f"   Orientacion FINAL: {orientacion} "
        "(90 grados = direccion +Y)"
    )
    print(
        f"2. Potencial: {c.potencial}; "
        f"ganancia={c.ganancia:g}; "
        f"transicion={c.transicion*100:g} cm"
    )
    print(
        f"3. Control: kv={c.kv:g}, kw={c.kw:g}; "
        f"avance max={c.vmax*100:g} cm/s"
    )
    print(
        f"   Giro max={math.degrees(c.wmax):g} grados/s; "
        f"rueda max={c.rueda_max*100:g} cm/s"
    )
    print(
        f"   Aceleracion={c.aceleracion*100:g} cm/s2; "
        f"frenado={c.frenado*100:g} cm/s2"
    )
    print(
        f"4. Llegada: error <= {c.tolerancia*100:g} cm; "
        f"error angular <= "
        f"{math.degrees(c.tolerancia_angular):g} grados"
    )
    print(
        f"   Periodo={c.periodo:g} s; "
        f"tiempo maximo={c.tiempo_max:g} s"
    )
    print(
        f"5. Robot: {c.robot}; "
        f"separacion de ruedas={c.separacion_ruedas*100:g} cm"
    )
    print(
        f"   Tiempo limite de comunicaciones="
        f"{c.timeout_lectura:g} s"
    )
    print(
        f"6. Repulsion: {c.repulsivo}; "
        f"ganancia={c.ganancia_repulsiva:g}; "
        f"influencia={c.influencia*100:g} cm"
    )
    print(
        "   Parametros registrados, SIN efecto en P01: "
        "escenario libre de obstaculos."
    )
    print("I. Iniciar mision con estos valores | Q. Salir sin conectar")
    print("0. Corregir la orientacion inicial introducida")


def editar_meta(c):
    x = pedir_numero("Meta X (cm)", c.meta_x*100) / 100
    y = pedir_numero("Meta Y (cm)", c.meta_y*100) / 100

    print(
        "Rumbo absoluto del escenario: "
        "0 = +X, 90 = +Y, 180 = -X, -90 = -Y."
    )

    while True:
        valor = input(
            "Orientacion final en grados "
            "(Intro conserva; 'no' desactiva): "
        ).strip().lower()

        try:
            theta = (
                c.meta_theta
                if not valor
                else None
                if valor == "no"
                else math.radians(float(valor.replace(",", ".")))
            )

            if theta is not None and not math.isfinite(theta):
                raise ValueError

            return replace(
                c,
                meta_x=x,
                meta_y=y,
                meta_theta=theta
            ).validar()

        except ValueError:
            print("Introduzca un angulo finito o 'no'.")


def editar_potencial(c):
    opciones = {
        "1": "cuadratico",
        "2": "conico",
        "3": "hibrido",
        "4": "suave"
    }

    print("1. Cuadratico: atraccion proporcional a la distancia.")
    print("2. Conico: magnitud constante fuera de la meta.")
    print("3. Hibrido: cuadratico cerca, conico lejos.")
    print(
        "4. Propio (suave): "
        "transicion gradual hasta una atraccion limitada."
    )

    while True:
        entrada = input(
            f"Potencial 1-4 [actual: {c.potencial}; Intro conserva]: "
        ).strip()

        if not entrada or entrada in opciones:
            potencial = opciones.get(entrada, c.potencial)
            break

        print("Elija una opcion del 1 al 4.")

    ganancia = pedir_numero(
        "Ganancia atractiva k (sus unidades dependen del potencial)",
        c.ganancia,
        True
    )

    transicion = c.transicion

    if potencial in ("hibrido", "suave"):
        transicion = pedir_numero(
            "Distancia de transicion atractiva (cm)",
            c.transicion*100,
            True
        ) / 100

    return replace(
        c,
        potencial=potencial,
        ganancia=ganancia,
        transicion=transicion
    ).validar()


def editar_parametros(c, campos):
    """La escala convierte unidades internas a las mostradas al usuario."""
    cambios = {}

    for nombre, texto, escala in campos:
        while True:
            valor = pedir_numero(
                texto,
                getattr(c, nombre)*escala,
                True
            ) / escala

            if nombre in ("vmax", "rueda_max") and valor > 0.50:
                print("El maximo admitido por el SDK es 50 cm/s.")
                continue

            cambios[nombre] = valor
            break

    return replace(c, **cambios).validar()


def menu_configuracion(c):
    while True:
        mostrar_configuracion(c)

        opcion = input(
            "Seleccione el apartado que desea editar: "
        ).strip().lower()

        if opcion == "0":
            c = replace(
                c,
                orientacion_inicial=pedir_orientacion_inicial()
            ).validar()

        elif opcion == "1":
            c = editar_meta(c)

        elif opcion == "2":
            c = editar_potencial(c)

        elif opcion == "3":
            c = editar_parametros(c, (
                ("kv", "Ganancia de avance kv", 1),
                ("kw", "Ganancia de giro kw", 1),
                ("vmax", "Velocidad maxima de avance (cm/s)", 100),
                (
                    "wmax",
                    "Velocidad maxima de giro (grados/s)",
                    180/math.pi
                ),
                ("rueda_max", "Velocidad maxima por rueda (cm/s)", 100),
                ("aceleracion", "Aceleracion de avance (cm/s2)", 100),
                ("frenado", "Frenado de aproximacion (cm/s2)", 100)
            ))

        elif opcion == "4":
            c = editar_parametros(c, (
                ("tolerancia", "Error de posicion permitido (cm)", 100),
                (
                    "tolerancia_angular",
                    "Error angular permitido (grados)",
                    180/math.pi
                ),
                ("periodo", "Periodo de control (s)", 1),
                ("tiempo_max", "Duracion maxima de la mision (s)", 1)
            ))

        elif opcion == "5":
            robot = input(
                f"Nombre Bluetooth [{c.robot}]: "
            ).strip() or c.robot

            c = editar_parametros(replace(c, robot=robot), (
                (
                    "separacion_ruedas",
                    "Separacion entre ruedas (cm)",
                    100
                ),
                (
                    "timeout_lectura",
                    "Tiempo limite de comunicaciones (s)",
                    1
                )
            ))

        elif opcion == "6":
            c = editar_repulsion(c)

        elif opcion == "i":
            print(
                "Coloque el robot en el origen, fuera de la base "
                "y en una zona SIN obstaculos."
            )

            confirmacion = input(
                "Escriba SI para conectar e iniciar, "
                "o Intro para volver: "
            ).strip().lower()

            if confirmacion == "si":
                return c.validar()

        elif opcion == "q":
            raise SystemExit("Mision cancelada sin conectar al robot.")

        else:
            print("Seleccione 0-6, I o Q.")


def editar_repulsion(c):
    """Completar el paso 2 sin introducir obstaculos ficticios en P01."""
    opciones = {
        "1": "cuadratico",
        "2": "inverso",
        "3": "lineal_tramos",
        "4": "propio"
    }

    print(
        "1. Cuadratico con influencia | 2. Inverso | "
        "3. Lineal por tramos | 4. Propio"
    )
    print(
        "En P01 solo se guarda la seleccion: "
        "no se leen IR ni se aplica repulsion."
    )

    while True:
        opcion = input(
            f"Funcion [actual: {c.repulsivo}; Intro conserva]: "
        ).strip()

        if not opcion or opcion in opciones:
            break

        print("Seleccione 1-4.")

    return editar_parametros(
        replace(c, repulsivo=opciones.get(opcion, c.repulsivo)),
        (
            (
                "ganancia_repulsiva",
                "Ganancia repulsiva (inactiva en P01)",
                1
            ),
            (
                "influencia",
                "Distancia de influencia (cm; inactiva en P01)",
                100
            )
        )
    )


def configurar():
    p = argparse.ArgumentParser(
        description="Parte 01: campo atractivo sin obstaculos."
    )

    p.add_argument("--robot", default="C3_UIEC_Grupo4")
    p.add_argument(
        "--x-cm",
        type=float,
        help="Meta X respecto al origen odometrico"
    )
    p.add_argument(
        "--y-cm",
        type=float,
        help="Meta Y en los ejes fijos del escenario"
    )
    p.add_argument(
        "--theta-grados",
        type=float,
        help="Orientacion final opcional"
    )
    p.add_argument(
        "--potencial",
        choices=["cuadratico", "conico", "hibrido", "suave"],
        default="hibrido"
    )
    p.add_argument("--ganancia", type=float, default=1.0)
    p.add_argument("--transicion-cm", type=float, default=30.0)
    p.add_argument("--kv", type=float, default=2.0)
    p.add_argument("--kw", type=float, default=1.5)
    p.add_argument("--vmax-cm-s", type=float, default=50.0)
    p.add_argument(
        "--wmax-grados-s",
        type=float,
        default=math.degrees(0.8)
    )
    p.add_argument("--rueda-max-cm-s", type=float, default=50.0)
    p.add_argument("--aceleracion-cm-s2", type=float, default=10.0)
    p.add_argument("--frenado-cm-s2", type=float, default=20.0)
    p.add_argument("--separacion-cm", type=float, default=23.5)
    p.add_argument("--tolerancia-cm", type=float, default=5.0)
    p.add_argument("--periodo", type=float, default=0.1)
    p.add_argument("--tiempo-max", type=float, default=120.0)
    p.add_argument("--tolerancia-angular-grados", type=float, default=3.0)
    p.add_argument("--timeout-lectura", type=float, default=2.0)
    p.add_argument(
        "--repulsivo",
        choices=["cuadratico", "inverso", "lineal_tramos", "propio"],
        default="cuadratico",
        help="Seleccion registrada; sin efecto en P01"
    )
    p.add_argument("--ganancia-repulsiva", type=float, default=0.002)
    p.add_argument("--influencia-cm", type=float, default=25)
    p.add_argument(
        "--sin-menu",
        action="store_true",
        help="Usar argumentos y confirmacion breve"
    )

    a = p.parse_args()

    try:
        # La orientacion inicial se escribe siempre.
        # Los argumentos precargan el resto del menu.
        orientacion_inicial = pedir_orientacion_inicial()

        x = (
            a.x_cm
            if a.x_cm is not None
            else pedir_numero("Meta X en el escenario (cm)", 0)
        )
        y = (
            a.y_cm
            if a.y_cm is not None
            else pedir_numero("Meta Y en el escenario (cm)", 100)
        )

        # Conversion a metros y radianes.
        c = Configuracion(
            robot=a.robot,
            meta_x=x/100,
            meta_y=y/100,
            orientacion_inicial=orientacion_inicial,
            meta_theta=(
                None
                if a.theta_grados is None
                else math.radians(a.theta_grados)
            ),
            potencial=a.potencial,
            ganancia=a.ganancia,
            transicion=a.transicion_cm/100,
            kv=a.kv,
            kw=a.kw,
            vmax=a.vmax_cm_s/100,
            wmax=math.radians(a.wmax_grados_s),
            rueda_max=a.rueda_max_cm_s/100,
            separacion_ruedas=a.separacion_cm/100,
            aceleracion=a.aceleracion_cm_s2/100,
            frenado=a.frenado_cm_s2/100,
            tolerancia=a.tolerancia_cm/100,
            periodo=a.periodo,
            tiempo_max=a.tiempo_max,
            tolerancia_angular=math.radians(a.tolerancia_angular_grados),
            timeout_lectura=a.timeout_lectura,
            repulsivo=a.repulsivo,
            ganancia_repulsiva=a.ganancia_repulsiva,
            influencia=a.influencia_cm/100
        ).validar()

    except (ValueError, EOFError) as error:
        p.error(str(error))

    if not a.sin_menu:
        return menu_configuracion(c)

    mostrar_configuracion(c)
    print(
        "Coloque el robot en el origen, fuera de la base "
        "y en una zona SIN obstaculos."
    )
    print(
        "La odometria se adapta al angulo inicial indicado; "
        "no se gira para alinearla."
    )
    input("Pulse Intro para conectar e iniciar; Ctrl+C cancela: ")

    return c


# CONTROL Y SISTEMAS DE REFERENCIA
@dataclass(frozen=True)
class Pose:
    x: float
    y: float
    theta: float


def angulo_corto(a):
    return math.atan2(math.sin(a), math.cos(a))


def pose_en_escenario(pose_sdk, origen_sdk, orientacion_inicial):
    """Trasladar y rotar la odometria a los ejes fijos del escenario.

    La primera pose define (0, 0). El desfase angular se conserva toda
    la mision: ni los ejes ni la meta giran cuando gira el robot.

    No mueve motores. Solo transforma coordenadas.
    Todas las entradas y salidas estan en metros y radianes.
    """
    desfase = orientacion_inicial - origen_sdk.theta

    dx = pose_sdk.x - origen_sdk.x
    dy = pose_sdk.y - origen_sdk.y

    c = math.cos(desfase)
    s = math.sin(desfase)

    return Pose(
        c*dx - s*dy,
        s*dx + c*dy,
        angulo_corto(pose_sdk.theta + desfase)
    )


def velocidades(fx, fy, distancia, config):
    """Avance y giro simultaneos, incluso cuando la meta queda detras.

    La atenuacion angular positiva reduce el avance en curvas cerradas
    sin convertir la navegacion en giro parado seguido de avance.
    """
    error = math.atan2(fy, fx)

    w = max(
        -config.wmax,
        min(config.wmax, config.kw*error)
    )

    # Reservar distancia para un ciclo de avance y el frenado:
    # v*Ts + v^2/(2*b) <= distancia a la meta.
    #
    # Usar la distancia completa evita tender a velocidad cero ANTES
    # de entrar en la tolerancia. Al entrar, calcular_ciclo ordena parar.
    disponible = max(0.0, distancia)
    bt = config.frenado * config.periodo

    limite_frenado = (
        math.sqrt(bt*bt + 2*config.frenado*disponible) - bt
    )

    v = min(
        config.vmax,
        config.kv*math.hypot(fx, fy),
        config.kv*distancia,
        limite_frenado
    )

    return v/(1.0 + error*error), w


class RampaAvance:
    """Memoria de la velocidad efectiva ordenada; independiente del SDK.

    Limita los aumentos. Las reducciones por curva, llegada o seguridad
    se aplican inmediatamente para respetar el limite calculado.

    El frenado es un modelo que debe contrastarse con el robot.
    """

    def __init__(self):
        self.actual = 0.0

    def limitar(self, objetivo, dt, config):
        # Un retraso Bluetooth no debe provocar un salto grande al reanudar.
        dt = max(0.0, min(dt, config.periodo))

        return min(
            objetivo,
            self.actual + config.aceleracion*dt
        )


def ruedas(v, w, config):
    """Velocidades tangenciales de ruedas en m/s.

    Si una rueda supera el limite se escalan ambas por el mismo factor
    para conservar la curvatura. Devuelve tambien las velocidades
    efectivas del robot: v y w.
    """
    izq = v - w*config.separacion_ruedas/2
    der = v + w*config.separacion_ruedas/2

    escala = max(
        1.0,
        abs(izq)/config.rueda_max,
        abs(der)/config.rueda_max
    )

    return izq/escala, der/escala, v/escala, w/escala


# POTENCIALES
def atractivo(dx, dy, config):
    """Devuelve (U, Fx, Fy) globales. dx,dy apuntan del robot a la meta.

    Propuesta propia:
        U = k*d0*(sqrt(d^2+d0^2)-d0)

    Su fuerza se satura suavemente lejos y tiende a cero al llegar.
    """
    d = math.hypot(dx, dy)

    if d == 0:
        return 0.0, 0.0, 0.0

    k = config.ganancia
    d0 = config.transicion

    if config.potencial == "cuadratico":
        u = 0.5*k*d*d
        magnitud = k*d

    elif config.potencial == "conico":
        # No diferenciable en cero: caso tratado arriba.
        u = k*d
        magnitud = k

    elif config.potencial == "hibrido":
        u = (
            0.5*k*d*d
            if d <= d0
            else k*d0*(d - 0.5*d0)
        )
        magnitud = k*min(d, d0)

    elif config.potencial == "suave":
        raiz = math.hypot(d, d0)
        u = k*d0*(raiz - d0)
        magnitud = k*d0*d/raiz

    else:
        raise ValueError("Potencial no implementado")

    return u, magnitud*dx/d, magnitud*dy/d


def a_local(fx, fy, theta):
    """R(theta)^T: eje X local delante, eje Y local a la izquierda."""
    c = math.cos(theta)
    s = math.sin(theta)

    return c*fx + s*fy, -s*fx + c*fy


# REGISTRO
class Registro:
    CAMPOS = (
        "t_s", "Ts_s", "dt_s", "x_m", "y_m", "theta_rad",
        "distancia_meta_m", "U_atr",
        "F_atr_x_local", "F_atr_y_local",
        "F_rep_x_local", "F_rep_y_local",
        "F_res_x_local", "F_res_y_local",
        "v_m_s", "w_rad_s", "v_izq_m_s", "v_der_m_s",
        "estado", "evento", "retraso_s", "v_odom_m_s", "w_odom_rad_s"
    )

    def __init__(self, config):
        self.carpeta = (
            Path(__file__).resolve().parent
            / "datos"
            / Path(__file__).stem
            / datetime.now().strftime("%Y%m%d_%H%M%S_%f")
        )
        self.carpeta.mkdir(parents=True)

        (self.carpeta / "configuracion.json").write_text(
            json.dumps(asdict(config), indent=2, ensure_ascii=False),
            encoding="utf-8"
        )

        self.archivo = (self.carpeta / "recorrido.csv").open(
            "w",
            newline="",
            encoding="utf-8"
        )
        self.csv = csv.DictWriter(
            self.archivo,
            fieldnames=self.CAMPOS
        )
        self.csv.writeheader()

        self.vmax = 0.0
        self.ciclos_tarde = 0
        self.plazos_omitidos = 0
        self.muestra_anterior = None
        self.vmax_odom = None

    def ciclo(
        self, t, dt, pose, distancia, potencial,
        fuerza, comando, estado, retraso, config
    ):
        izq, der, v, w = comando
        fx, fy = fuerza

        self.vmax = max(self.vmax, abs(v))

        # Estimacion por diferencias de odometria.
        # No es una medida externa de la velocidad.
        v_odom = w_odom = None

        if self.muestra_anterior is not None:
            t0, p0 = self.muestra_anterior
            intervalo = t - t0

            if intervalo > 0:
                v_odom = math.hypot(
                    pose.x - p0.x,
                    pose.y - p0.y
                ) / intervalo

                w_odom = angulo_corto(
                    pose.theta - p0.theta
                ) / intervalo

                self.vmax_odom = max(
                    self.vmax_odom or 0.0,
                    v_odom
                )

        self.muestra_anterior = (t, pose)
        self.ciclos_tarde += int(dt > config.periodo*1.05)

        self.csv.writerow(dict(zip(self.CAMPOS, (
            t, config.periodo, dt,
            pose.x, pose.y, pose.theta,
            distancia, potencial,
            fx, fy, 0, 0, fx, fy,
            v, w, izq, der,
            estado, "", retraso,
            v_odom, w_odom
        ))))
        self.archivo.flush()

    def evento(self, t, motivo):
        self.csv.writerow({
            "t_s": t,
            "estado": "FIN",
            "evento": motivo
        })
        self.archivo.flush()

    def cerrar(self, motivo, tiempo, error):
        self.archivo.close()

        datos = {
            "resultado": motivo,
            "tiempo_s": tiempo,
            "error_final_m": error,
            "vmax_ordenada_m_s": self.vmax,
            "ciclos_retrasados": self.ciclos_tarde,
            "vmax_estimada_odometria_m_s": self.vmax_odom,
            "plazos_omitidos": self.plazos_omitidos,
            "distancia_minima_obstaculos_m": None,
            "nota": (
                "Parte 01 sin IR. Velocidad ordenada y estimada "
                "por diferencias de odometria; no medida externa. "
                "Ciclo lento: dt > 1.05*Ts."
            )
        }

        (self.carpeta / "resultado.json").write_text(
            json.dumps(datos, indent=2, ensure_ascii=False),
            encoding="utf-8"
        )


# MISION
def calcular_ciclo(pose, config, rampa=None, dt=0.0):
    """Calcular el estado y el comando de un ciclo sin acceder al SDK."""
    dx = config.meta_x - pose.x
    dy = config.meta_y - pose.y

    distancia = math.hypot(dx, dy)
    u, fx, fy = atractivo(dx, dy, config)
    fuerza = a_local(fx, fy, pose.theta)

    if distancia <= config.tolerancia:
        error = (
            0
            if config.meta_theta is None
            else angulo_corto(config.meta_theta - pose.theta)
        )

        if abs(error) <= config.tolerancia_angular:
            if rampa is not None:
                rampa.actual = 0.0

            return "LLEGADA", distancia, u, fuerza, (0, 0, 0, 0)

        # Giro sobre el eje solo para la orientacion final solicitada,
        # una vez alcanzada la posicion de destino.
        v = 0
        w = max(
            -config.wmax,
            min(config.wmax, config.kw*error)
        )
        estado = "ORIENTACION_FINAL"

    else:
        v, w = velocidades(*fuerza, distancia, config)

        if rampa is not None:
            v = rampa.limitar(v, dt, config)

        estado = "NAVEGACION"

    comando = ruedas(v, w, config)

    if rampa is not None:
        # Guardar la velocidad efectiva tras escalar las ruedas.
        rampa.actual = comando[2]

    return estado, distancia, u, fuerza, comando


async def navegar(adaptador, config):
    registro = Registro(config)
    rampa = RampaAvance()

    inicio = anterior = siguiente = time.monotonic()
    motivo, error_final = "interrumpida", None

    print(f"Registro de esta ejecucion: {registro.carpeta}")

    try:
        while True:
            adaptador.comprobar_seguridad()
            ahora = time.monotonic()

            if ahora - inicio >= config.tiempo_max:
                raise TimeoutError("Tiempo maximo de mision alcanzado")

            pose = await adaptador.leer_pose()

            # Fecha de recepcion de la muestra de odometria.
            ahora = time.monotonic()

            estado, error_final, u, fuerza, comando = calcular_ciclo(
                pose,
                config,
                rampa,
                ahora - anterior
            )

            await adaptador.aplicar(comando[0], comando[1])

            registro.ciclo(
                ahora - inicio,
                ahora - anterior,
                pose,
                error_final,
                u,
                fuerza,
                comando,
                estado,
                max(0, ahora - siguiente),
                config
            )

            anterior = ahora

            if estado == "LLEGADA":
                motivo = "meta_alcanzada"
                break

            siguiente += config.periodo

            # Esperar al plazo del ciclo.
            # Si vamos tarde, omitir plazos vencidos sin rafagas de comandos.
            actual = time.monotonic()

            if siguiente < actual:
                omitidos = (
                    math.floor((actual - siguiente)/config.periodo) + 1
                )
                registro.plazos_omitidos += omitidos
                siguiente += omitidos*config.periodo

            await asyncio.sleep(
                max(0, siguiente - time.monotonic())
            )

    except BaseException as exc:
        motivo = f"{type(exc).__name__}: {exc}"
        raise

    finally:
        # Parar antes de escribir el resumen.
        # Intentar cerrar el registro aunque falle la parada por Bluetooth.
        try:
            await adaptador.parar()
        finally:
            try:
                registro.evento(
                    time.monotonic() - inicio,
                    motivo
                )
            finally:
                registro.cerrar(
                    motivo,
                    time.monotonic() - inicio,
                    error_final
                )

    return motivo


# ADAPTADOR DEL ROBOT
class Adaptador:
    def __init__(self, robot, config):
        self.robot = robot
        self.config = config
        self.colision = False
        self.origen_sdk = None

    def comprobar_seguridad(self):
        if self.colision:
            raise RuntimeError("Parachoques activado; mision cancelada")

    async def leer_pose(self):
        self.comprobar_seguridad()

        p = await asyncio.wait_for(
            self.robot.get_position(),
            self.config.timeout_lectura
        )

        if p is None or not all(
            math.isfinite(a) for a in (p.x, p.y, p.heading)
        ):
            raise RuntimeError("Odometria ausente o no valida")

        # El SDK reutiliza Pose: copiar sus valores.
        # Convertir centimetros y grados a metros y radianes.
        pose_sdk = Pose(
            p.x/100,
            p.y/100,
            math.radians(p.heading)
        )

        if self.origen_sdk is None:
            # Primera lectura con el robot parado tras reset_navigation.
            self.origen_sdk = pose_sdk

        # Convertir la pose al escenario definido por el equipo.
        return pose_en_escenario(
            pose_sdk,
            self.origen_sdk,
            self.config.orientacion_inicial
        )

    async def aplicar(self, izq, der):
        self.comprobar_seguridad()

        await asyncio.wait_for(
            self.robot.set_wheel_speeds(100*izq, 100*der),
            self.config.timeout_lectura
        )

        # Si el bumper interrumpio el envio, asegurar velocidad cero.
        if self.colision:
            await self.parar()
            self.comprobar_seguridad()

    async def parar(self):
        await asyncio.wait_for(
            self.robot.set_wheel_speeds(0, 0),
            self.config.timeout_lectura
        )


def ejecutar(config):
    from irobot_edu_sdk.backend.bluetooth import Bluetooth
    from irobot_edu_sdk.robots import Create3, event

    config.validar()

    robot = Create3(Bluetooth(config.robot))
    adaptador = Adaptador(robot, config)

    @event(robot.when_bumped, [])
    async def colision(r):
        adaptador.colision = True
        await adaptador.parar()
        await r.set_lights_blink_rgb(255, 0, 0)

    @event(robot.when_play)
    async def iniciar(r):
        try:
            await adaptador.parar()

            await asyncio.wait_for(
                r.reset_navigation(),
                config.timeout_lectura
            )
            await asyncio.sleep(0.2)

            await r.set_lights_on_rgb(0, 255, 0)

            # Esta primera lectura fija el origen.
            # No se ordena ningun giro de alineacion inicial.
            await adaptador.leer_pose()

            await r.set_lights_on_rgb(0, 0, 255)
            resultado = await navegar(adaptador, config)
            print(resultado)

            await r.set_lights_on_rgb(255, 80, 0)

        except Exception as exc:
            print(f"Mision detenida: {exc}")

        finally:
            try:
                await adaptador.parar()
            finally:
                try:
                    await asyncio.wait_for(
                        r.stop(),
                        config.timeout_lectura
                    )
                finally:
                    await asyncio.wait_for(
                        r.disconnect(),
                        config.timeout_lectura
                    )

    # El SDK gestiona su bucle: no envolver play() en asyncio.run().
    robot.play()


def main():
    ejecutar(configurar())


if __name__ == "__main__":
    main()

    
    # =============================================================================
# P01 - COMPARACION DE POTENCIALES ATRACTIVOS
# =============================================================================
# Fecha: ____________________    Equipo: ____________________
#
# CONDICIONES COMUNES (colocar el robot en el mismo inicio en cada prueba)
# Meta: X = ______ cm | Y = ______ cm | Orientacion final = ______ grados / NO
# Velocidad maxima: ______ cm/s | Periodo de control: ______ s
# Tolerancia de posicion: ______ cm | Tolerancia angular: ______ grados
# Ganancia de avance kv: ______ | Ganancia de giro kw: ______
#
# | Potencial  | Ganancia k | Transicion (cm) | Tiempo (s) | Error odom. (cm) | Error real (cm) | Vmax odom. (cm/s) |
# |------------|------------|-----------------|------------|------------------|-----------------|-------------------|
# | Cuadratico |            | No aplica       |            |                  |                 |                   |
# | Conico     |            | No aplica       |            |                  |                 |                   |
# | Hibrido    |            |                 |            |                  |                 |                   |
# | Suave      |            |                 |            |                  |                 |                   |
#
# OBSERVACIONES
# Cuadratico:
#   ¿Llego? ______ | Trayectoria, suavidad, oscilaciones: ______________________
#   Carpeta del registro: ___________________________________________________
#
# Conico:
#   ¿Llego? ______ | Trayectoria, suavidad, oscilaciones: ______________________
#   Carpeta del registro: ___________________________________________________
#
# Hibrido:
#   ¿Llego? ______ | Trayectoria, suavidad, oscilaciones: ______________________
#   Carpeta del registro: ___________________________________________________
#
# Suave (propuesta propia):
#   ¿Llego? ______ | Trayectoria, suavidad, oscilaciones: ______________________
#   Carpeta del registro: ___________________________________________________
#
# ORIGEN DE LOS DATOS
# - Tiempo: resultado.json -> tiempo_s.
# - Error odometrico: resultado.json -> error_final_m, multiplicado por 100.
# - Error real: distancia desde la referencia del robot hasta la meta,
#   medida con cinta usando la misma referencia que al marcar el inicio.
# - Vmax odometrica: resultado.json -> vmax_estimada_odometria_m_s x 100.
#   Es una estimacion por odometria, no una medida externa.
# - Si un dato no esta disponible, escribir N/D; no sustituirlo por cero.
#
# CONCLUSION
# Potencial que funciono mejor y motivo:
# __________________________________________________________________________
# __________________________________________________________________________
# =============================================================================