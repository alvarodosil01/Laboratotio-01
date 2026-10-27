"""Parte 01: atraccion. Grupo 4. Programa autonomo organizado por responsabilidades."""
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

    # Paso 2: parametros seleccionables y registrados,
    # inactivos en el mapa libre de P01.
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
            "timeout_lectura", "ganancia_repulsiva", "influencia"
        ):
            if getattr(self, nombre) <= 0:
                raise ValueError(f"{nombre}: debe ser mayor que cero")

        if self.potencial not in ("cuadratico", "conico", "hibrido", "suave"):
            raise ValueError("Potencial desconocido")

        if not self.robot.strip():
            raise ValueError("Falta el nombre Bluetooth")

        if self.repulsivo not in (
            "cuadratico", "inverso", "lineal_tramos", "propio"
        ):
            raise ValueError("Seleccion repulsiva desconocida")

        return self


def pedir_numero(texto, actual, positivo=False):
    """Aceptar Intro para conservar el valor y repetir si no es valido."""
    while True:
        entrada = input(f"{texto} [{actual:g}]: ").strip().replace(",", ".")

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


def mostrar_configuracion(c):
    print("\n--- P01: configuracion de la mision ---")
    print(
        "Origen: posicion inicial. "
        "X+ derecha, X- izquierda; Y+ delante, Y- detras."
    )
    print(f"1. Meta: X={c.meta_x*100:g} cm, Y={c.meta_y*100:g} cm")

    orientacion = (
        "sin orientacion final"
        if c.meta_theta is None
        else f"{math.degrees(c.meta_theta):g} grados"
    )

    print(f"   Orientacion: {orientacion} (90 grados = rumbo inicial)")
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
        f"4. Llegada: error <= {c.tolerancia*100:g} cm; "
        f"error angular <= {math.degrees(c.tolerancia_angular):g} grados"
    )
    print(
        f"   Periodo={c.periodo:g} s; "
        f"tiempo maximo={c.tiempo_max:g} s"
    )
    print(
        f"5. Robot: {c.robot}; "
        f"separacion de ruedas={c.separacion_ruedas*100:g} cm"
    )
    print(f"   Tiempo limite de comunicaciones={c.timeout_lectura:g} s")
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


def editar_meta(c):
    x = pedir_numero("Meta X (cm)", c.meta_x*100)/100
    y = pedir_numero("Meta Y (cm)", c.meta_y*100)/100

    print("Rumbo absoluto: 0 derecha, 90 delante, 180 izquierda, -90 detras.")

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
    print("4. Propio (suave): transicion gradual hasta una atraccion limitada.")

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
        )/100

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
        cambios[nombre] = pedir_numero(
            texto,
            getattr(c, nombre)*escala,
            True
        )/escala

    return replace(c, **cambios).validar()


def menu_configuracion(c):
    while True:
        mostrar_configuracion(c)

        opcion = input(
            "Seleccione el apartado que desea editar: "
        ).strip().lower()

        if opcion == "1":
            c = editar_meta(c)

        elif opcion == "2":
            c = editar_potencial(c)

        elif opcion == "3":
            c = editar_parametros(c, (
                ("kv", "Ganancia de avance kv", 1),
                ("kw", "Ganancia de giro kw", 1),
                ("vmax", "Velocidad maxima de avance (cm/s)", 100),
                ("wmax", "Velocidad maxima de giro (grados/s)", 180/math.pi),
                ("rueda_max", "Velocidad maxima por rueda (cm/s)", 100)
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
                ("separacion_ruedas", "Separacion entre ruedas (cm)", 100),
                ("timeout_lectura", "Tiempo limite de comunicaciones (s)", 1)
            ))

        elif opcion == "6":
            c = editar_repulsion(c)

        elif opcion == "i":
            print(
                "Coloque el robot en el origen, fuera de la base "
                "y en una zona SIN obstaculos."
            )

            confirmacion = input(
                "Escriba SI para conectar e iniciar, o Intro para volver: "
            ).strip().lower()

            if confirmacion == "si":
                return c.validar()

        elif opcion == "q":
            raise SystemExit("Mision cancelada sin conectar al robot.")

        else:
            print("Seleccione 1-6, I o Q.")


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
            ("ganancia_repulsiva", "Ganancia repulsiva (inactiva en P01)", 1),
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
        help="Meta Y; el SDK inicia mirando a +Y"
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
    p.add_argument("--kv", type=float, default=0.4)
    p.add_argument("--kw", type=float, default=1.5)
    p.add_argument("--vmax-cm-s", type=float, default=5.0)
    p.add_argument(
        "--wmax-grados-s",
        type=float,
        default=math.degrees(0.8)
    )
    p.add_argument("--rueda-max-cm-s", type=float, default=10.0)
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
        # Los argumentos precargan el menu.
        # La conversion a unidades SI se realiza aqui.
        x = (
            a.x_cm
            if a.x_cm is not None
            else pedir_numero("Meta X (cm): + derecha, - izquierda", 0)
        )
        y = (
            a.y_cm
            if a.y_cm is not None
            else pedir_numero("Meta Y (cm): + delante, - detras", 100)
        )

        c = Configuracion(
            robot=a.robot,
            meta_x=x/100,
            meta_y=y/100,
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
        "Al reiniciar la odometria: "
        "X a la derecha, Y hacia delante (rumbo 90 grados)."
    )
    input("Pulse Intro para conectar e iniciar; Ctrl+C cancela: ")

    return c


# CONTROL
@dataclass(frozen=True)
class Pose:
    x: float
    y: float
    theta: float


def angulo_corto(a):
    return math.atan2(math.sin(a), math.cos(a))


def velocidades(fx, fy, distancia, config):
    """Avance y giro simultaneos, incluso cuando la meta queda detras.

    La atenuacion angular positiva reduce el avance en curvas cerradas sin
    convertir la navegacion en un giro parado seguido de avance. El limite
    por distancia suaviza la llegada incluso con el potencial conico.
    """
    error = math.atan2(fy, fx)

    w = max(
        -config.wmax,
        min(config.wmax, config.kw*error)
    )
    v = min(
        config.vmax,
        config.kv*math.hypot(fx, fy),
        config.kv*distancia
    )

    return v/(1.0+error*error), w


def ruedas(v, w, config):
    """Velocidades tangenciales (m/s), no angulares: no se usa radio de rueda.

    Si una rueda supera el limite se escalan ambas por el mismo factor para
    conservar la curvatura. Devuelve tambien las velocidades efectivas v,w.
    """
    izq = v-w*config.separacion_ruedas/2
    der = v+w*config.separacion_ruedas/2

    escala = max(
        1.0,
        abs(izq)/config.rueda_max,
        abs(der)/config.rueda_max
    )

    return izq/escala, der/escala, v/escala, w/escala


# POTENCIALES
def atractivo(dx, dy, config):
    """Devuelve (U, Fx, Fy) globales. dx,dy apuntan del robot a la meta.

    La propuesta suave usa U=k*d0*(sqrt(d^2+d0^2)-d0). Su fuerza se
    satura suavemente lejos de la meta y tiende a cero al llegar.
    """
    d = math.hypot(dx, dy)

    if d == 0:
        return 0.0, 0.0, 0.0

    k, d0 = config.ganancia, config.transicion

    if config.potencial == "cuadratico":
        u, magnitud = 0.5*k*d*d, k*d

    elif config.potencial == "conico":
        # No diferenciable en cero: caso tratado arriba.
        u, magnitud = k*d, k

    elif config.potencial == "hibrido":
        u = (
            0.5*k*d*d
            if d <= d0
            else k*d0*(d-0.5*d0)
        )
        magnitud = k*min(d, d0)

    elif config.potencial == "suave":
        raiz = math.hypot(d, d0)
        u = k*d0*(raiz-d0)
        magnitud = k*d0*d/raiz

    else:
        raise ValueError("Potencial no implementado")

    return u, magnitud*dx/d, magnitud*dy/d


def a_local(fx, fy, theta):
    """R(theta)^T: eje X local hacia delante, Y local hacia la izquierda."""
    c, s = math.cos(theta), math.sin(theta)
    return c*fx+s*fy, -s*fx+c*fy


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

        # Intervalos entre lecturas reales, no entre comandos:
        # estimacion por odometria, no una medida externa.
        v_odom = w_odom = None

        if self.muestra_anterior is not None:
            t0, p0 = self.muestra_anterior
            intervalo = t-t0

            if intervalo > 0:
                v_odom = math.hypot(
                    pose.x-p0.x,
                    pose.y-p0.y
                )/intervalo

                w_odom = angulo_corto(
                    pose.theta-p0.theta
                )/intervalo

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
def calcular_ciclo(pose, config):
    """Estado y comando de un ciclo, comprobable sin conectar el robot."""
    dx = config.meta_x-pose.x
    dy = config.meta_y-pose.y

    distancia = math.hypot(dx, dy)
    u, fx, fy = atractivo(dx, dy, config)
    fuerza = a_local(fx, fy, pose.theta)

    if distancia <= config.tolerancia:
        error = (
            0
            if config.meta_theta is None
            else angulo_corto(config.meta_theta-pose.theta)
        )

        if abs(error) <= config.tolerancia_angular:
            return "LLEGADA", distancia, u, fuerza, (0, 0, 0, 0)

        # Unicamente al llegar se permite el giro final sobre el eje.
        v = 0
        w = max(
            -config.wmax,
            min(config.wmax, config.kw*error)
        )
        estado = "ORIENTACION_FINAL"

    else:
        v, w = velocidades(*fuerza, distancia, config)
        estado = "NAVEGACION"

    return estado, distancia, u, fuerza, ruedas(v, w, config)


async def navegar(adaptador, config):
    registro = Registro(config)
    inicio = anterior = siguiente = time.monotonic()
    motivo, error_final = "interrumpida", None

    print(f"Registro de esta ejecucion: {registro.carpeta}")

    try:
        while True:
            adaptador.comprobar_seguridad()
            ahora = time.monotonic()

            if ahora-inicio >= config.tiempo_max:
                raise TimeoutError("Tiempo maximo de mision alcanzado")

            pose = await adaptador.leer_pose()

            # Fecha de recepcion de la muestra de odometria.
            ahora = time.monotonic()

            estado, error_final, u, fuerza, comando = calcular_ciclo(
                pose,
                config
            )

            await adaptador.aplicar(comando[0], comando[1])

            registro.ciclo(
                ahora-inicio,
                ahora-anterior,
                pose,
                error_final,
                u,
                fuerza,
                comando,
                estado,
                max(0, ahora-siguiente),
                config
            )

            anterior = ahora

            if estado == "LLEGADA":
                motivo = "meta_alcanzada"
                break

            siguiente += config.periodo

            # No sumar Ts al tiempo de calculo: esperar al plazo del ciclo.
            # Si vamos tarde, omitir plazos vencidos sin rafagas de comandos.
            actual = time.monotonic()

            if siguiente < actual:
                omitidos = (
                    math.floor((actual-siguiente)/config.periodo)+1
                )
                registro.plazos_omitidos += omitidos
                siguiente += omitidos*config.periodo

            await asyncio.sleep(
                max(0, siguiente-time.monotonic())
            )

    except BaseException as exc:
        motivo = f"{type(exc).__name__}: {exc}"
        raise

    finally:
        # Parar antes de escribir el resumen; intentar cerrar el registro
        # incluso si Bluetooth falla al enviar la parada.
        try:
            await adaptador.parar()
        finally:
            try:
                registro.evento(
                    time.monotonic()-inicio,
                    motivo
                )
            finally:
                registro.cerrar(
                    motivo,
                    time.monotonic()-inicio,
                    error_final
                )

    return motivo


# ROBOT
class Adaptador:
    def __init__(self, robot, config):
        self.robot = robot
        self.config = config
        self.colision = False

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

        # El SDK reutiliza Pose: copiar los valores.
        return Pose(
            p.x/100,
            p.y/100,
            math.radians(p.heading)
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