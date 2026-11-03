"""LRM_01_P03 - Laboratorio Robots Moviles 01, Parte 03: minimos locales.

Asignatura: Robots Autonomos (UIE). Profesor: Eladio Dapena Gonzalez.
Equipo: Grupo 4 (robot C3_UIEC_Grupo4).
Integrantes: COMPLETAR (nombre y apellidos de cada integrante).

Piloto reactivo con campos de potencial para el iRobot Create 3 que lleva el
robot de qi a qf evitando obstaculos y que detecta y escapa de minimos locales
(por ejemplo, un obstaculo en U entre qi y qf). Parte de LRM_01_P02.py.

Cadena del piloto (P01 + P02):
    Percepcion: odometria (x, y, theta) y 7 IR -> distancias calibradas,
        memoria de puntos en ejes fijos (repulsion tambien en la zona ciega).
    U_atr -> F_atr (cuadratico, conico, hibrido o propio "suave").
    U_rep,i -> F_rep,i (Khatib o polinomico) con corredor de paso: lo que
        bloquea la trayectoria se rodea por un subobjetivo lateral; lo lateral
        solo empuja de lado (paso entre dos obstaculos por el medio).
    F_res -> (v, w) simultaneos -> (v_izq, v_der) con saturacion proporcional.

Supervisor de minimos locales (Parte 03), maquina de estados:

          +-------------+  sin progreso y rodeado de   +-------------------+
          | NAVEGACION  |  paredes (minimo local)      | ESCAPE_CONTORNO   |
          | (piloto P02)| ---------------------------> | o ESCAPE_SUBMETA  |
          |             |                              |  -> CAMINO_MAPA   |
          |             | <--------------------------- | (camino del mapa  |
          +-------------+  meta cerca o tiempo agotado |  hasta la meta)   |
                 |                                     +-------------------+
                 |                                       (max. max_escapes)
                 v d_f <= epsilon
          LLEGADA / ORIENTACION_FINAL

    Detector: en una ventana de ventana_minimo segundos la distancia a la meta
    no mejora progreso_minimo (parada, oscilacion o giro en el sitio) Y el
    robot esta rodeado de paredes (dentro de la envolvente convexa del mapa a
    RADIO_CONCAVIDAD): rodeando un objeto suelto tambien se pierde progreso,
    pero no se esta dentro de un obstaculo convexo. Separa las 29 detecciones
    de las pruebas reales: todas las de la U si, todas las falsas no. No
    exige F_res = 0 exacto: con ruido y ciclos de ~0.35 s el robot oscila
    alrededor del minimo en vez de pararse en el.
    Mapa temporal: rejilla de 5 cm con todo lo que ven los IR en la mision
        (no olvida, a diferencia de la memoria de 10 s de la repulsion).
    Estrategia 1, contorno con el mapa: A* sobre el mapa desde el robot a la
        meta, a distancia_pared de las paredes conocidas; lo no recorrido
        cuesta mas, asi que sale de la U por donde entro y la rodea por fuera
        pegado a sus bordes. Se rehace cada REPLANIFICAR s si los IR ven
        paredes nuevas, y se sigue HASTA LA META (el piloto P02 no usa el
        mapa y volvia a meterse en la U). A v_cerca solo dentro de la U.
    Estrategia 2, submetas: sale de la U deshaciendo su propio rastro hasta
        RETROCESO m y va a una submeta a 90 grados de la meta, a un lado, lo
        justo para librar la U (al menos distancia_submeta); de ahi a la meta
        por el camino del mapa (estado CAMINO_MAPA, igual que el contorno). El
        lado es por donde la U del mapa sobresale menos.
    Un intento fallido (tiempo_max_escape) cambia de lado en el siguiente.
    "ninguna" desactiva la recuperacion: sirve de evidencia del problema (Paso 2).

Luces: azul navegando; amarillo al detectar un minimo local hasta salir de la U;
naranja al llegar; rojo parpadeante si falla o choca.

Uso: python LRM_01_P03.py y menu (o --sin-menu con argumentos). Registro por
ejecucion en datos/LRM_01_P03/<fecha>/: recorrido.csv, configuracion.json,
calibracion.json, resultado.json (tiempo, error, escapes, distancia minima) y
mapa.csv (celdas ocupadas del mapa temporal).
"""
import argparse
import asyncio
import csv
import json
import heapq
import math
import time
from collections import deque
from dataclasses import dataclass, asdict, replace
from datetime import datetime
from pathlib import Path


# Geometria nominal Create 3, en orden IZQUIERDA -> DERECHA del SDK
# (side_left, left, front_left, front_center, front_right, right, side_right).
# Fuente: iRobotEducation/create3_sim, create3.urdf.xacro (rama jazzy).
# https://github.com/iRobotEducation/create3_sim/blob/jazzy/irobot_create_common/irobot_create_description/urdf/create3.urdf.xacro
# Radianes LOCALES respecto al frontal (+ = izquierda). No son orientaciones globales.
# Orden comprobado en el registro 20261102_181647: al girar a la derecha el
# obstaculo paso del indice 5 al 0, luego el indice 0 es el de la izquierda.
# Con el orden invertido la repulsion lateral empujaba HACIA el obstaculo.
ANGULOS_IR = (1.309, 0.873, 0.436, 0.0, -0.436, -0.873, -1.309)
ESTRATEGIAS = ("ninguna", "contorno", "submeta")


@dataclass(frozen=True)
class Configuracion:
    robot: str = "C3_UIEC_Grupo4"
    meta_x: float = 0.0
    meta_y: float = 1.0
    meta_theta: float | None = None
    orientacion_inicial: float = math.pi/2  # Respecto a los ejes FIJOS del escenario.
    potencial: str = "hibrido"
    ganancia: float = 1.0
    transicion: float = 0.30
    kv: float = 2.0
    kw: float = 1.5
    # Con lecturas en paralelo el ciclo real es ~0.3 s (registro 20261103_100830).
    # En espacio libre limite_por_percepcion acota a ~30 cm/s. Limite SDK 50 cm/s.
    vmax: float = 0.50
    wmax: float = 1.5
    rueda_max: float = 0.50
    aceleracion: float = 0.30  # m/s2: aumento progresivo desde reposo.
    frenado: float = 0.50  # m/s2: parametro del perfil de aproximacion.
    separacion_ruedas: float = 0.235  # Heredada de T03: medir antes de validar.
    tolerancia: float = 0.05  # Con 3 cm daba vueltas alrededor de la meta.
    tolerancia_angular: float = math.radians(3)
    periodo: float = 0.10
    tiempo_max: float = 180.0
    timeout_lectura: float = 2.0
    # Repulsion ACTIVA: campos de la etapa P02.
    repulsivo: str = "khatib"
    ganancia_repulsiva: float = 0.03  # 0.004 no superaba la atraccion: choque frontal.
    influencia: float = 0.25
    # Componente tangencial: rodear el obstaculo; 0 desactiva (repulsion pura).
    ganancia_tangencial: float = 1.0
    liberar_esquiva: float = 1.0  # s sin deteccion para olvidar el lado de rodeo.
    memoria_obstaculo: float = 10.0  # s que se recuerda cada punto IR fuera de la vista.

    angulos_ir: tuple = ANGULOS_IR  # Geometria fija al chasis; no se pide por teclado.
    # Con ciclos reales de ~0.4 s y alcance IR de ~15 cm, cada ciclo de retraso
    # (filtro o confirmacion) supone ~5 cm mas cerca: por defecto, sin retraso.
    filtro_alpha: float = 1.0
    confirmaciones_ir: int = 1  # Lecturas consecutivas por sensor para activar repulsion.
    distancia_giro: float = 0.20  # IR frontal por debajo: girar en el sitio.
    v_cerca: float = 0.12  # Avance maximo con un obstaculo DELANTE (+-50 grados).
    presencia_ir: float = 0.70  # Fraccion del umbral IR frontal que ya frena.
    histeresis_ir: float = 0.10  # Desactivar al caer por debajo del 90% del umbral.
    seguridad: float = 0.06
    fuerza_max_sensor: float = 2.0
    calibracion_json: str = ""  # Vacio: tabla heredada de sesion 04/T03.
    # Parte 03: supervisor de minimos locales. Solo la estrategia se elige en
    # el menu; el resto son valores fijos, probados en el robot y en simulacion.
    estrategia_escape: str = "contorno"  # "ninguna", "contorno" o "submeta".
    ventana_minimo: float = 6.0  # s sin progreso para declarar minimo local.
    progreso_minimo: float = 0.10  # m que debe mejorar la distancia en la ventana.
    distancia_pared: float = 0.12  # m (IR) a las paredes en el camino del mapa.
    margen_salida: float = 0.05  # m mas cerca de la meta que al detectar (Bug2).
    distancia_submeta: float = 0.6  # m minimos de la submeta lateral al punto de retroceso.
    tiempo_max_escape: float = 90.0  # s por intento (salida, rodeo y, en contorno, hasta la meta).
    max_escapes: int = 5

    def validar(self):
        for nombre, valor in asdict(self).items():
            if isinstance(valor, (int, float)) and not math.isfinite(valor):
                raise ValueError(f"{nombre}: debe ser finito")
        for nombre in ("ganancia", "transicion", "kv", "kw", "vmax", "wmax",
                       "rueda_max", "separacion_ruedas", "tolerancia",
                       "tolerancia_angular", "periodo", "tiempo_max", "timeout_lectura",
                       "ganancia_repulsiva", "influencia", "aceleracion", "frenado", "seguridad", "fuerza_max_sensor",
                       "liberar_esquiva", "memoria_obstaculo", "ventana_minimo",
                       "progreso_minimo", "distancia_pared", "margen_salida",
                       "distancia_submeta", "tiempo_max_escape"):
            if getattr(self, nombre) <= 0:
                raise ValueError(f"{nombre}: debe ser mayor que cero")
        if self.ganancia_tangencial < 0:
            raise ValueError("ganancia_tangencial: no puede ser negativa")
        if self.potencial not in ("cuadratico", "conico", "hibrido", "suave"):
            raise ValueError("Potencial desconocido")
        if not self.robot.strip():
            raise ValueError("Falta el nombre Bluetooth")
        if self.vmax > 0.50 or self.rueda_max > 0.50:
            raise ValueError("El SDK limita el avance y cada rueda a 50 cm/s")
        if self.repulsivo not in ("khatib", "polinomico"):
            raise ValueError("Seleccion repulsiva desconocida")
        validar_angulos(self.angulos_ir)
        if not 0 < self.filtro_alpha <= 1:
            raise ValueError("El filtro alpha debe cumplir 0 < alpha <= 1")
        if not isinstance(self.confirmaciones_ir, int) or self.confirmaciones_ir < 1:
            raise ValueError("Las confirmaciones IR deben ser un entero positivo")
        if not 0 <= self.histeresis_ir < 1:
            raise ValueError("La histeresis IR debe estar entre 0 y 1 (excluido)")
        if self.seguridad >= self.influencia:
            raise ValueError("La distancia de seguridad debe ser menor que la influencia")
        if not self.seguridad < self.distancia_giro <= self.influencia:
            raise ValueError("Se requiere seguridad < distancia de giro <= influencia")
        if self.v_cerca <= 0 or not 0 < self.presencia_ir <= 1:
            raise ValueError("v_cerca debe ser positiva y 0 < presencia_ir <= 1")
        if self.estrategia_escape not in ESTRATEGIAS:
            raise ValueError("Estrategia de escape desconocida")
        if not self.seguridad < self.distancia_pared < self.influencia:
            raise ValueError("Se requiere seguridad < distancia a la pared < influencia")
        if self.ventana_minimo <= self.periodo:
            raise ValueError("La ventana del detector debe superar el periodo de control")
        if not isinstance(self.max_escapes, int) or self.max_escapes < 1:
            raise ValueError("max_escapes debe ser un entero positivo")
        return self


def pedir_numero(texto, actual, positivo=False):
    """Aceptar Intro para conservar el valor y repetir si la entrada no es valida."""
    while True:
        entrada = input(f"{texto} [{actual:g}]: ").strip().replace(",", ".")
        try:
            valor = actual if not entrada else float(entrada)
            if not math.isfinite(valor) or (positivo and valor <= 0):
                raise ValueError
            return valor
        except ValueError:
            print("Introduzca un numero finito" + (" mayor que cero." if positivo else "."))


def pedir_orientacion_inicial():
    """Exigir un angulo escrito; no suponer que el robot mira hacia +Y."""
    print("Ejes fijos del suelo: 0 grados = +X; 90 = +Y; 180 = -X; -90 = -Y.")
    print("El robot empieza en (0, 0), pero puede mirar en cualquier direccion.")
    while True:
        entrada = input("Escriba la orientacion INICIAL del robot (grados): ").strip()
        try:
            valor = float(entrada.replace(",", "."))
            if not math.isfinite(valor):
                raise ValueError
            return math.radians(valor)
        except ValueError:
            print("Es obligatorio escribir un angulo finito, por ejemplo 45 o -90.")


def mostrar_configuracion(c):
    print("\n--- P03: configuracion de la mision ---")
    print("Origen: (0, 0) inicial. X e Y son ejes fijos marcados en el suelo.")
    print(f"   Orientacion INICIAL: {math.degrees(c.orientacion_inicial):g} grados")
    print(f"1. Meta: X={c.meta_x*100:g} cm, Y={c.meta_y*100:g} cm")
    orientacion = "sin orientacion final" if c.meta_theta is None else f"{math.degrees(c.meta_theta):g} grados"
    print(f"   Orientacion FINAL: {orientacion} (90 grados = direccion +Y)")
    print(f"2. Potencial: {c.potencial}; ganancia={c.ganancia:g}; transicion={c.transicion*100:g} cm")
    print(f"3. Control: kv={c.kv:g}, kw={c.kw:g}; avance max={c.vmax*100:g} cm/s")
    print(f"   Giro max={math.degrees(c.wmax):g} grados/s; rueda max={c.rueda_max*100:g} cm/s")
    print(f"   Aceleracion={c.aceleracion*100:g} cm/s2; frenado={c.frenado*100:g} cm/s2")
    print(f"4. Llegada: error <= {c.tolerancia*100:g} cm; error angular <= {math.degrees(c.tolerancia_angular):g} grados")
    print(f"   Periodo={c.periodo:g} s; tiempo maximo={c.tiempo_max:g} s")
    print(f"5. Robot: {c.robot}; separacion de ruedas={c.separacion_ruedas*100:g} cm")
    print(f"   Tiempo limite de comunicaciones={c.timeout_lectura:g} s")
    print(f"6. Repulsion: {c.repulsivo}; ganancia={c.ganancia_repulsiva:g}; influencia={c.influencia*100:g} cm")
    print(f"   Fuerza maxima por sensor={c.fuerza_max_sensor:g}")
    print(f"   Rodeo tangencial={c.ganancia_tangencial:g}; liberar lado tras {c.liberar_esquiva:g} s sin deteccion")
    print(f"   Memoria de obstaculos fuera de la vista={c.memoria_obstaculo:g} s")
    print(f"7. Sensores: alpha={c.filtro_alpha:g}; seguridad={c.seguridad*100:g} cm")
    print(f"   Giro en el sitio si IR frontal < {c.distancia_giro*100:g} cm; avance con obstaculo <= {c.v_cerca*100:g} cm/s")
    print("   Geometria IR fija al robot; acompana automaticamente su orientacion.")
    print(f"   Deteccion: {c.confirmaciones_ir} lecturas consecutivas; histeresis={c.histeresis_ir*100:g}%")
    print("   Calibracion:", c.calibracion_json or "tabla heredada de sesion 04/T03")
    print(f"8. Estrategia ante minimos locales: {c.estrategia_escape}")
    print("I. Iniciar mision con estos valores | Q. Salir sin conectar")
    print("0. Corregir la orientacion inicial introducida")


def editar_meta(c):
    x = pedir_numero("Meta X (cm)", c.meta_x*100)/100
    y = pedir_numero("Meta Y (cm)", c.meta_y*100)/100
    print("Rumbo absoluto del escenario: 0 = +X, 90 = +Y, 180 = -X, -90 = -Y.")
    while True:
        valor = input("Orientacion final en grados (Intro conserva; 'no' desactiva): ").strip().lower()
        try:
            theta = c.meta_theta if not valor else None if valor == "no" else math.radians(float(valor.replace(",", ".")))
            if theta is not None and not math.isfinite(theta):
                raise ValueError
            return replace(c, meta_x=x, meta_y=y, meta_theta=theta).validar()
        except ValueError:
            print("Introduzca un angulo finito o 'no'.")


def editar_potencial(c):
    opciones = {"1": "cuadratico", "2": "conico", "3": "hibrido", "4": "suave"}
    print("1. Cuadratico: atraccion proporcional a la distancia.")
    print("2. Conico: magnitud constante fuera de la meta.")
    print("3. Hibrido: cuadratico cerca, conico lejos.")
    print("4. Propio (suave): transicion gradual hasta una atraccion limitada.")
    while True:
        entrada = input(f"Potencial 1-4 [actual: {c.potencial}; Intro conserva]: ").strip()
        if not entrada or entrada in opciones:
            potencial = opciones.get(entrada, c.potencial)
            break
        print("Elija una opcion del 1 al 4.")
    ganancia = pedir_numero("Ganancia atractiva k (sus unidades dependen del potencial)", c.ganancia, True)
    transicion = c.transicion
    if potencial in ("hibrido", "suave"):
        transicion = pedir_numero("Distancia de transicion atractiva (cm)", c.transicion*100, True)/100
    return replace(c, potencial=potencial, ganancia=ganancia, transicion=transicion).validar()


def editar_parametros(c, campos):
    """La escala convierte unidades internas a las unidades mostradas al usuario."""
    cambios = {}
    for nombre, texto, escala in campos:
        while True:
            valor = pedir_numero(texto, getattr(c, nombre)*escala, True)/escala
            if nombre in ("vmax", "rueda_max") and valor > 0.50:
                print("El maximo admitido por el SDK es 50 cm/s.")
                continue
            cambios[nombre] = valor
            break
    try:
        return replace(c, **cambios).validar()
    except ValueError as exc:
        print(f"No se guardaron los cambios: {exc}")
        return c


def menu_configuracion(c):
    while True:
        mostrar_configuracion(c)
        opcion = input("Seleccione el apartado que desea editar: ").strip().lower()
        if opcion == "0":
            c = replace(c, orientacion_inicial=pedir_orientacion_inicial()).validar()
        elif opcion == "1":
            c = editar_meta(c)
        elif opcion == "2":
            c = editar_potencial(c)
        elif opcion == "3":
            c = editar_parametros(c, (
                ("kv", "Ganancia de avance kv", 1), ("kw", "Ganancia de giro kw", 1),
                ("vmax", "Velocidad maxima de avance (cm/s)", 100),
                ("wmax", "Velocidad maxima de giro (grados/s)", 180/math.pi),
                ("rueda_max", "Velocidad maxima por rueda (cm/s)", 100),
                ("aceleracion", "Aceleracion de avance (cm/s2)", 100),
                ("frenado", "Frenado de aproximacion (cm/s2)", 100)))
        elif opcion == "4":
            c = editar_parametros(c, (
                ("tolerancia", "Error de posicion permitido (cm)", 100),
                ("tolerancia_angular", "Error angular permitido (grados)", 180/math.pi),
                ("periodo", "Periodo de control (s)", 1),
                ("tiempo_max", "Duracion maxima de la mision (s)", 1)))
        elif opcion == "5":
            robot = input(f"Nombre Bluetooth [{c.robot}]: ").strip() or c.robot
            c = editar_parametros(replace(c, robot=robot), (
                ("separacion_ruedas", "Separacion entre ruedas (cm)", 100),
                ("timeout_lectura", "Tiempo limite de comunicaciones (s)", 1)))
        elif opcion == "6":
            c = editar_repulsion(c)
        elif opcion == "7":
            c = editar_sensores(c)
        elif opcion == "8":
            c = editar_minimos(c)
        elif opcion == "i":
            try:
                calibrador = cargar_calibracion(c)
            except (ValueError, TypeError, OSError) as exc:
                print(f"Revise la calibracion y los parametros: {exc}")
                continue
            print("Coloque el robot en el origen indicado y verifique el escenario de P03.")
            if input("Escriba SI para conectar e iniciar, o Intro para volver: ").strip().lower() == "si":
                return c.validar(), calibrador
        elif opcion == "q":
            raise SystemExit("Mision cancelada sin conectar al robot.")
        else:
            print("Seleccione 0-8, I o Q.")


def editar_minimos(c):
    print("1. Ninguna: sin recuperacion (evidencia del minimo local, Paso 2).")
    print("2. Contorno: mapa temporal de lo visto y camino (A*) que rodea la U por sus bordes.")
    print("3. Submeta: retroceder por el camino recorrido, ir a un lado y volver a la meta.")
    opciones = {"1": "ninguna", "2": "contorno", "3": "submeta"}
    while True:
        entrada = input(f"Estrategia 1-3 [actual: {c.estrategia_escape}; Intro conserva]: ").strip()
        if not entrada or entrada in opciones:
            break
        print("Elija una opcion del 1 al 3.")
    return replace(c, estrategia_escape=opciones.get(entrada, c.estrategia_escape)).validar()


def configurar():
    p = argparse.ArgumentParser(description="Parte 03: campos de potencial con deteccion y escape de minimos locales.")
    p.add_argument("--robot", default="C3_UIEC_Grupo4")
    p.add_argument("--x-cm", type=float, help="Meta X respecto al origen odometrico")
    p.add_argument("--y-cm", type=float, help="Meta Y en los ejes fijos del escenario")
    p.add_argument("--theta-grados", type=float, help="Orientacion final opcional")
    p.add_argument("--potencial", choices=["cuadratico", "conico", "hibrido", "suave"], default="hibrido")
    p.add_argument("--ganancia", type=float, default=1.0)
    p.add_argument("--transicion-cm", type=float, default=30.0)
    p.add_argument("--kv", type=float, default=2.0)
    p.add_argument("--kw", type=float, default=1.5)
    p.add_argument("--vmax-cm-s", type=float, default=30.0)
    p.add_argument("--wmax-grados-s", type=float, default=math.degrees(1.5))
    p.add_argument("--rueda-max-cm-s", type=float, default=50.0)
    p.add_argument("--aceleracion-cm-s2", type=float, default=30.0)
    p.add_argument("--frenado-cm-s2", type=float, default=50.0)
    p.add_argument("--separacion-cm", type=float, default=23.5)
    p.add_argument("--tolerancia-cm", type=float, default=5.0)
    p.add_argument("--periodo", type=float, default=0.1)
    p.add_argument("--tiempo-max", type=float, default=180.0, help="Rodear una U lleva tiempo")
    p.add_argument("--tolerancia-angular-grados", type=float, default=3.0)
    p.add_argument("--timeout-lectura", type=float, default=2.0)
    p.add_argument("--repulsivo", choices=["khatib", "polinomico"], default="khatib", help="Funcion repulsiva activa")
    p.add_argument("--ganancia-repulsiva", "--krep", type=float, default=0.03)
    p.add_argument("--influencia-cm", type=float, default=25)
    p.add_argument("--ganancia-tangencial", "--ktan", type=float, default=1.0,
                   help="Rodeo del obstaculo; 0 = repulsion pura")
    p.add_argument("--liberar-esquiva", type=float, default=1.0)
    p.add_argument("--memoria-s", type=float, default=10.0, help="Segundos que se recuerda un obstaculo")
    p.add_argument("--sin-menu", action="store_true", help="Usar argumentos y confirmacion breve")
    p.add_argument("--alpha", type=float, default=1.0)
    p.add_argument("--seguridad-cm", type=float, default=6)
    p.add_argument("--giro-cm", type=float, default=20, help="IR frontal que provoca giro en el sitio")
    p.add_argument("--v-cerca-cm-s", type=float, default=12, help="Avance maximo con obstaculo delante")
    p.add_argument("--fuerza-max-sensor", type=float, default=2)
    p.add_argument("--calibracion-json", default="", help="Tabla {distancia_cm: [ir1,...,ir7]}")
    p.add_argument("--escape", choices=ESTRATEGIAS, default="contorno",
                   help="Estrategia ante un minimo local; 'ninguna' = sin recuperacion")
    a = p.parse_args()
    try:
        # Los argumentos precargan el menu; la conversion a SI se realiza aqui.
        orientacion_inicial = pedir_orientacion_inicial()
        x = a.x_cm if a.x_cm is not None else pedir_numero("Meta X en el escenario (cm)", 0)
        y = a.y_cm if a.y_cm is not None else pedir_numero("Meta Y en el escenario (cm)", 100)
        c = Configuracion(robot=a.robot, meta_x=x / 100, meta_y=y / 100,
            orientacion_inicial=orientacion_inicial,
            meta_theta=None if a.theta_grados is None else math.radians(a.theta_grados),
            potencial=a.potencial, ganancia=a.ganancia, transicion=a.transicion_cm / 100,
            kv=a.kv, kw=a.kw, vmax=a.vmax_cm_s / 100, wmax=math.radians(a.wmax_grados_s),
            rueda_max=a.rueda_max_cm_s / 100, separacion_ruedas=a.separacion_cm / 100,
            aceleracion=a.aceleracion_cm_s2/100, frenado=a.frenado_cm_s2/100,
            tolerancia=a.tolerancia_cm / 100, periodo=a.periodo, tiempo_max=a.tiempo_max,
            tolerancia_angular=math.radians(a.tolerancia_angular_grados),
            timeout_lectura=a.timeout_lectura, repulsivo=a.repulsivo,
            ganancia_repulsiva=a.ganancia_repulsiva, influencia=a.influencia_cm/100,
            ganancia_tangencial=a.ganancia_tangencial, liberar_esquiva=a.liberar_esquiva,
            memoria_obstaculo=a.memoria_s,
            filtro_alpha=a.alpha, seguridad=a.seguridad_cm/100,
            distancia_giro=a.giro_cm/100, v_cerca=a.v_cerca_cm_s/100,
            fuerza_max_sensor=a.fuerza_max_sensor, calibracion_json=a.calibracion_json,
            estrategia_escape=a.escape).validar()
    except (ValueError, EOFError) as error:
        p.error(str(error))
    if not a.sin_menu:
        return menu_configuracion(c)
    try:
        calibrador = cargar_calibracion(c)
    except (ValueError, TypeError, OSError) as exc:
        p.error(str(exc))
    mostrar_configuracion(c)
    print("Coloque el robot en el origen, fuera de la base, en el escenario de P03.")
    print("La odometria se adapta al angulo inicial indicado; no se gira para alinearla.")
    input("Pulse Intro para conectar e iniciar; Ctrl+C cancela: ")
    return c, calibrador


@dataclass(frozen=True)
class Pose:
    x: float
    y: float
    theta: float


def angulo_corto(a):
    return math.atan2(math.sin(a), math.cos(a))


def pose_en_escenario(pose_sdk, origen_sdk, orientacion_inicial):
    """Trasladar y rotar la odometria a los ejes fijos elegidos por el equipo.

    La primera pose define (0, 0). El desfase angular se conserva toda la
    mision: ni los ejes ni la meta giran cuando gira el robot. No mueve motores.
    Todas las entradas y salidas estan en metros y radianes.
    """
    desfase = orientacion_inicial-origen_sdk.theta
    dx, dy = pose_sdk.x-origen_sdk.x, pose_sdk.y-origen_sdk.y
    c, s = math.cos(desfase), math.sin(desfase)
    return Pose(c*dx-s*dy, s*dx+c*dy,
                angulo_corto(pose_sdk.theta+desfase))


def velocidades(fx, fy, distancia, config, w_previo=0.0, retraso=0.0):
    """Avance y giro simultaneos, incluso cuando la meta queda detras.

    La atenuacion angular positiva reduce el avance en curvas cerradas sin
    convertir la navegacion en un giro parado seguido de avance. El limite
    por distancia suaviza la llegada incluso con el potencial conico.

    Cada orden actua ~1 ciclo tarde (registro 20261103_102021: con 14 grados
    de error el robot siguio girando a 1.7 rad/s y se paso 45 grados: trazado
    en S). Se descuenta el giro que aun hara la orden anterior durante
    `retraso` y no se pide mas giro del que falta en un ciclo.
    """
    error = math.atan2(fy, fx)
    previsto = angulo_corto(error-w_previo*retraso)
    w = max(-config.wmax, min(config.wmax, config.kw*previsto))
    if retraso > 0:
        w = max(-abs(previsto)/retraso, min(abs(previsto)/retraso, w))
    # Reservar distancia para un ciclo de avance y el frenado:
    # v*Ts + v^2/(2*b) <= distancia a la meta. Usar la distancia completa
    # evita tender a velocidad cero ANTES de entrar en la tolerancia.
    # Al entrar en ella, calcular_ciclo ordena parar.
    disponible = max(0.0, distancia)
    bt = config.frenado*config.periodo
    limite_frenado = math.sqrt(bt*bt+2*config.frenado*disponible)-bt
    v = min(config.vmax, config.kv*math.hypot(fx, fy),
            config.kv*distancia, limite_frenado)
    return v/(1.0+error*error), w


class RampaAvance:
    """Memoria de la velocidad efectiva ordenada; independiente del SDK.

    Limita los aumentos. Una reduccion por curva, llegada o seguridad se aplica
    inmediatamente: no se conserva una velocidad superior al limite calculado.
    El parametro de frenado es un modelo que debe contrastarse con el robot.
    """
    def __init__(self):
        self.actual = 0.0
        self.giro = 0.0  # Ultimo w ordenado: aun no reflejado en la odometria.

    def limitar(self, objetivo, dt, config):
        # Un retraso Bluetooth no debe provocar un salto grande al reanudar.
        dt = max(0.0, min(dt, config.periodo))
        return min(objetivo, self.actual+config.aceleracion*dt)


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


INTENSIDAD_MINIMA = 20.0  # Por debajo, ruido del sensor y no un obstaculo.

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
        if len(self.muestras) < 2:
            raise ValueError("Se requieren al menos dos distancias calibradas")
        if any(not math.isfinite(d) or d <= 0 for d in self.muestras):
            raise ValueError("Distancias de calibracion invalidas")
        for fila in self.muestras.values():
            if len(fila) != 7 or any(not math.isfinite(v) or v <= 0 for v in fila):
                raise ValueError("Cada distancia requiere 7 intensidades positivas y finitas")
        self.tablas = [tuple((fila[i], d/100) for d, fila in self.muestras.items()) for i in range(7)]
        if any(any(a[0] <= b[0] for a, b in zip(t, t[1:])) for t in self.tablas):
            raise ValueError("La intensidad de cada sensor debe disminuir con la distancia")
        # Umbral de deteccion por sensor: el extremo de la tabla, pero nunca
        # por debajo del ruido. El IR 6 marca 15.6 a 20 cm y 15.1 a 25 cm, su
        # nivel de ruido (lee hasta 17 sin nada delante): el ruido aparecia
        # como obstaculo a 19 cm, la memoria lo guardaba 10 s y apartaba al
        # robot de la meta (20261103_113635).
        self.umbrales = tuple(max(t[-1][0], INTENSIDAD_MINIMA) for t in self.tablas)

    def validar_configuracion(self, config):
        minimo, maximo = min(self.muestras)/100, max(self.muestras)/100
        if not minimo < config.seguridad < config.influencia <= maximo:
            raise ValueError(f"La tabla exige {minimo*100:g} cm < seguridad < influencia <= {maximo*100:g} cm")
        return self

    def convertir(self, valores):
        if len(valores) != 7 or any(v is None or not math.isfinite(v) or v < 0 for v in valores):
            raise RuntimeError("Lectura IR invalida; se detiene la mision")
        return tuple(self.distancia(v, t, u) for v, t, u in zip(valores, self.tablas, self.umbrales))

    @staticmethod
    def distancia(valor, tabla, umbral):
        if valor < umbral:
            return math.inf  # Sin deteccion util, no una medida de 50 cm.
        if valor >= tabla[0][0]:
            return tabla[0][1]  # Saturacion cercana; activa la parada de seguridad.
        for (a, da), (b, db) in zip(tabla, tabla[1:]):
            if b <= valor <= a:
                return da+(a-valor)/(a-b)*(db-da)
        raise RuntimeError("No se pudo interpolar la lectura IR")


class DetectorIR:
    """Confirmar presencia por sensor sin modificar las medidas de seguridad.

    Entrada: dos muestras seguidas sobre el umbral calibrado de alcance.
    Salida: intensidad inferior al umbral menos el margen de histeresis.
    En la banda inferior, sin distancia calibrada, usar el extremo del rango
    solo como entrada del filtro (no como una nueva medida ni extrapolacion).
    Esto deja decaer la repulsion sin alternar presencia por pequenas oscilaciones.
    """
    def __init__(self, calibrador, config):
        self.config = config
        self.umbrales = calibrador.umbrales
        self.alcance = max(calibrador.muestras)/100
        self.cuentas = [0]*7
        self.activas = [False]*7

    def actualizar(self, intensidades, distancias):
        confirmadas = []
        for i, (valor, distancia, umbral) in enumerate(zip(intensidades, distancias, self.umbrales)):
            if self.activas[i]:
                if valor < umbral*(1-self.config.histeresis_ir):
                    self.activas[i] = False
                    self.cuentas[i] = 0
            else:
                self.cuentas[i] = self.cuentas[i]+1 if valor >= umbral else 0
                if self.cuentas[i] >= self.config.confirmaciones_ir:
                    self.activas[i] = True
            confirmadas.append(
                min(distancia, self.alcance) if self.activas[i] else math.inf)
        return tuple(confirmadas)


class FiltroIR:
    def __init__(self, alpha):
        self.alpha, self.anteriores = alpha, None

    def actualizar(self, distancias):
        # Recibe detecciones ya confirmadas con histeresis. Al desactivarse un
        # sensor, borrar su contribucion sin dejar repulsion residual.
        # Nunca mezclar infinito y una distancia en la media exponencial.
        if self.anteriores is None:
            self.anteriores = distancias
        self.anteriores = tuple(
            d if not math.isfinite(d) or not math.isfinite(prev)
            else self.alpha*d+(1-self.alpha)*prev
            for d, prev in zip(distancias, self.anteriores))
        return self.anteriores


def magnitud_repulsiva(distancia, config):
    """(U, |F|) de un punto; None fuera de la influencia o sin deteccion."""
    if not math.isfinite(distancia) or distancia >= config.influencia:
        return None
    d, d0, k = max(distancia, 0.005), config.influencia, config.ganancia_repulsiva
    if config.repulsivo == "khatib":
        u, magnitud = 0.5*k*(1/d-1/d0)**2, k*(1/d-1/d0)/d**2
    else:
        u, magnitud = 0.5*k*(1-d/d0)**2, k*(1-d/d0)/d0
    return u, min(magnitud, config.fuerza_max_sensor)


def bloquea(distancia, angulo):
    """Punto por delante y dentro del corredor que barre el robot al avanzar."""
    return math.cos(angulo) > 0 and abs((RADIO_IR+distancia)*math.sin(angulo)) <= CORREDOR


def repulsion(distancias, config, angulos=None):
    """Khatib: U=k/2*(1/d-1/d0)^2; alternativa U=k/2*(1-d/d0)^2.

    Ambas son cero fuera de d0. Fuerza = -grad(U), hacia fuera del obstaculo.
    La magnitud de cada fuerza se limita como regularizacion del controlador.
    Sus ganancias tienen distintas unidades: ajustar por separado.
    Sin angulos se usan los de los IR; con ellos, los de la memoria.

    Corredor de paso: un punto por delante cuya separacion lateral supera
    CORREDOR, o uno de detras, no esta en la trayectoria; solo empuja de lado
    (centra el robot entre dos obstaculos) y no frena ni empuja hacia delante.
    En un paso estrecho los dos bordes frenaban mas que la atraccion y el
    robot giraba sin entrar (20261103_105824). Devuelve (U, Fx, Fy, Bx, By):
    B es la parte de los puntos que SI bloquean la trayectoria.
    """
    fx = fy = bx = by = potencial = 0.0
    for distancia, angulo in zip(distancias, config.angulos_ir if angulos is None else angulos):
        punto = magnitud_repulsiva(distancia, config)
        if punto is None:
            continue
        potencial += punto[0]
        magnitud = punto[1]
        px, py = magnitud*math.cos(angulo), magnitud*math.sin(angulo)
        if bloquea(distancia, angulo):
            bx, by = bx-px, by-py
        else:
            # Fuera del corredor o detras: solo empuje lateral. Los puntos de
            # detras recordados empujaban hacia delante (~3 frente a 0.3 de
            # atraccion) y el robot huia del paso tras girar en el sitio.
            px = 0.0
        fx, fy = fx-px, fy-py
    return potencial, fx, fy, bx, by


RADIO_IR = 0.154  # m del centro a los emisores IR (URDF): origen de cada distancia.
RADIO_ROBOT = 0.17  # m: radio del chasis Create 3.
CORREDOR = RADIO_ROBOT+0.03  # m: semiancho barrido al avanzar.


def antes_de_meta(distancia_ir, distancia_meta):
    """La distancia IR si el punto puede tocarse de camino a la meta; si no, inf."""
    return distancia_ir if RADIO_IR+distancia_ir < distancia_meta+RADIO_ROBOT else math.inf
CELDA_MEMORIA = 0.03  # m: puntos mas proximos se funden en uno.
SECTOR_MEMORIA = math.radians(30)  # Un sensor virtual por sector alrededor del robot.


class MemoriaObstaculos:
    """Puntos IR detectados, guardados en los ejes FIJOS con la odometria.

    Los IR solo cubren +-75 grados: al rodear un obstaculo este queda en la
    zona ciega, la repulsion desaparece y la atraccion devuelve el robot
    contra el (vaiven registrado en 20261102_182733). Recordar los puntos
    mantiene la repulsion en los 360 grados. Se devuelve el punto mas cercano
    de cada sector, como sensores virtuales, para que los muchos puntos de un
    mismo obstaculo no sumen fuerzas sin limite. Un LIDAR podria alimentarla igual.
    """
    def __init__(self):
        self.puntos = {}  # celda -> (x, y, t)

    def actualizar(self, pose, distancias, t, config):
        for d, a in zip(distancias, config.angulos_ir):
            if math.isfinite(d):
                r, g = RADIO_IR+d, pose.theta+a
                x, y = pose.x+r*math.cos(g), pose.y+r*math.sin(g)
                self.puntos[(round(x/CELDA_MEMORIA), round(y/CELDA_MEMORIA))] = (x, y, t)
        self.puntos = {c: p for c, p in self.puntos.items() if t-p[2] <= config.memoria_obstaculo}

    def virtuales(self, pose):
        """Listas (distancias, angulos locales) del punto mas cercano por sector."""
        sectores = {}
        for x, y, _ in self.puntos.values():
            dx, dy = x-pose.x, y-pose.y
            d = math.hypot(dx, dy)-RADIO_IR
            angulo = angulo_corto(math.atan2(dy, dx)-pose.theta)
            s = round(angulo/SECTOR_MEMORIA)
            if d < sectores.get(s, (math.inf, 0))[0]:
                sectores[s] = (d, angulo)
        return [d for d, _ in sectores.values()], [a for _, a in sectores.values()]


def elegir_lado(rep, atr):
    """+1 rodea por la izquierda y -1 por la derecha (marco local).

    Primero el lado libre (la repulsion empuja hacia el); si el obstaculo
    esta centrado, el lado de la meta; si tambien lo esta, la izquierda.
    """
    if abs(rep[1]) > 0.2*math.hypot(*rep):
        return 1 if rep[1] > 0 else -1
    if abs(atr[1]) > 1e-3:
        return 1 if atr[1] > 0 else -1
    return 1


class MemoriaEsquiva:
    """Lado de rodeo elegido al detectar el obstaculo y mantenido hasta dejarlo atras.

    Frente a un obstaculo centrado la repulsion se opone a la atraccion sin
    componente lateral: el robot solo frena y choca. Fijar el lado evita
    ademas dudar entre izquierda y derecha mientras lo rodea.
    """
    def __init__(self):
        self.lado, self.libre = 0, 0.0
        self.girando = False  # Giro en el sitio por proximidad, con histeresis.
        self.memoria = MemoriaObstaculos()

    def actualizar(self, rep, atr, dt, config):
        if rep != (0.0, 0.0):
            self.libre = 0.0
            if self.lado == 0:
                self.lado = elegir_lado(rep, atr)
        elif self.lado:
            self.libre += dt
            if self.libre >= config.liberar_esquiva:
                self.lado, self.libre = 0, 0.0
        return self.lado


def tangencial(distancias, angulos, config):
    """F_rep de cada punto lateral girada 90 grados: bordear el obstaculo.

    Cada punto se deja en su lado (a la derecha se rodea por la izquierda y
    viceversa); con un obstaculo a cada lado ambos empujan hacia delante y
    el robot pasa por el medio. Con un lado fijo para todo, la pared del otro
    lado empujaba HACIA ATRAS y el robot giraba sin pasar. Elegir el sentido
    hacia la meta se probo en simulacion y alternaba de lado en cada ciclo.
    Los puntos que bloquean no cuentan: los resuelve desvio().
    """
    tx = ty = 0.0
    for distancia, angulo in zip(distancias, angulos):
        punto = magnitud_repulsiva(distancia, config)
        if punto is None or bloquea(distancia, angulo):
            continue
        sentido = 1 if math.sin(angulo) < 0 else -1
        k = sentido*config.ganancia_tangencial*punto[1]
        tx -= k*math.sin(angulo)
        ty += k*math.cos(angulo)
    return tx, ty


def desvio(distancias, angulos, lado, config):
    """Rumbo local que deja cada punto que bloquea a CORREDOR del eje de avance.

    Subobjetivo virtual: en vez de empujar contra el obstaculo, la atraccion
    apunta a pasar por su lado (lado +1: izquierda). Para un punto a angulo a
    y distancia r del centro: a + lado*asin(CORREDOR/r); con varios, el mas
    exigente. None si nada bloquea. Con un sensor de mas alcance (LIDAR) el
    desvio empezaria antes y el giro seria suave.
    """
    rumbo = None
    for distancia, angulo in zip(distancias, angulos):
        if magnitud_repulsiva(distancia, config) is None or not bloquea(distancia, angulo):
            continue
        r = RADIO_IR+distancia
        psi = angulo+lado*math.asin(min(1.0, CORREDOR/r))
        if rumbo is None or lado*(psi-rumbo) > 0:
            rumbo = psi
    return rumbo


WMAX_ESCAPE = 0.8  # rad/s: giro maximo en el escape (curvas, no trompos).
GIRO_ESCAPE = 0.16  # m: frontal que obliga a girar en el sitio en el escape.


def via_libre(distancias, angulos, rumbo, config):
    """Ningun punto bloquea el corredor del robot si avanza con ese rumbo local."""
    return not any(magnitud_repulsiva(d, config) is not None and bloquea(d, a-rumbo)
                   for d, a in zip(distancias, angulos))


CELDA_MAPA = 0.05  # m: lado de la celda del mapa temporal.
ZONA_MAPA = 0.15  # m mas alla de la holgura con coste: pasar algo mas lejos de las paredes.
COSTE_ZONA = 3.0  # Coste extra maximo junto a la holgura.
COSTE_DESCONOCIDO = 1.0  # Coste extra de lo no recorrido DENTRO de la U: salir por donde
# se entro. Fuera de ella sin coste: con el, al salir seguia su rastro hacia atras, giraba
# hacia un lado y luego cambiaba al otro (20261103_173601).
ANCHO_RASTRO = 0.15  # m alrededor del centro que el chasis (radio 0.17) ha barrido: libre.
MARGEN_MAPA = 1.0  # m alrededor de lo conocido donde se busca camino.
MAX_EXPANSIONES = 60000
VECINOS = tuple((dx, dy, math.hypot(dx, dy)) for dx in (-1, 0, 1) for dy in (-1, 0, 1) if dx or dy)


class MapaTemporal:
    """Mapa de ocupacion temporal: rejilla de CELDA_MAPA en ejes fijos con
    TODO lo que detectan los IR durante la mision, y el rastro recorrido.

    La memoria de la repulsion olvida a los 10 s; este mapa no: sabe donde
    esta la U para rodearla. Al seguir el camino planificado los IR ven
    paredes nuevas (la cara exterior de la U) y el camino se rehace.
    """
    def __init__(self):
        self.ocupadas = set()
        self.libres = set()  # Celdas barridas por el chasis: seguro que libres.
        self.version = 0  # Cambia cuando aparece una celda ocupada nueva.

    @staticmethod
    def celda(x, y):
        return round(x/CELDA_MAPA), round(y/CELDA_MAPA)

    def actualizar(self, pose, distancias, angulos):
        # Donde ha estado el chasis no hay obstaculo: esas celdas se borran y
        # no se vuelven a marcar. Lecturas IR falsas sobre el camino de
        # entrada cerraban la boca de la U en el mapa y el A* no encontraba
        # salida (20261103_171100).
        cx, cy = self.celda(pose.x, pose.y)
        k = round(ANCHO_RASTRO/CELDA_MAPA)
        barridas = {(cx+dx, cy+dy) for dx in range(-k, k+1) for dy in range(-k, k+1)
                    if dx*dx+dy*dy <= k*k}
        self.libres |= barridas
        antes = set(self.ocupadas)
        self.ocupadas -= barridas
        for d, a in zip(distancias, angulos):
            if math.isfinite(d):
                r, g = RADIO_IR+d, pose.theta+a
                celda = self.celda(pose.x+r*math.cos(g), pose.y+r*math.sin(g))
                if celda not in self.libres:
                    self.ocupadas.add(celda)
        if self.ocupadas != antes:
            self.version += 1

    def camino(self, inicio, meta, holgura, zona_u=(), lado=None):
        """A* (8 vecinos) de inicio a meta; puntos (x, y) en m, o None.

        El robot es un punto: celdas a menos de holgura (radio + distancia a
        la pared) de algo ocupado, prohibidas; hasta ZONA_MAPA mas, con coste
        para no rozar. Lo desconocido se supone libre pero cuesta mas que el
        rastro: sale de la U por donde entro y la rodea por fuera, pegado a
        la holgura de sus paredes. Alrededor del inicio se permite pasar por
        celdas prohibidas (el robot puede estar ya cerca de una pared).
        """
        k = math.ceil((holgura+ZONA_MAPA)/CELDA_MAPA)
        coste = {}
        for dx in range(-k, k+1):
            for dy in range(-k, k+1):
                d = math.hypot(dx, dy)*CELDA_MAPA
                if d <= holgura+ZONA_MAPA:
                    coste[(dx, dy)] = math.inf if d <= holgura else COSTE_ZONA*(1-(d-holgura)/ZONA_MAPA)
        extra = {}
        for cx, cy in self.ocupadas:
            for (dx, dy), c in coste.items():
                n = (cx+dx, cy+dy)
                if c > extra.get(n, 0.0):
                    extra[n] = c
        s, g = self.celda(*inicio), self.celda(*meta)
        if extra.get(g, 0.0) == math.inf:
            return None
        xs = [c[0] for c in self.ocupadas]+[s[0], g[0]]
        ys = [c[1] for c in self.ocupadas]+[s[1], g[1]]
        m = round(MARGEN_MAPA/CELDA_MAPA)
        x0, x1, y0, y1 = min(xs)-m, max(xs)+m, min(ys)-m, max(ys)+m
        cerca_inicio = (holgura/CELDA_MAPA)**2
        abiertos, g_coste, padre, expandidas = [(0.0, s)], {s: 0.0}, {}, 0
        while abiertos and expandidas < MAX_EXPANSIONES:
            _, c = heapq.heappop(abiertos)
            if c == g:
                break
            expandidas += 1
            for dx, dy, paso in VECINOS:
                n = (c[0]+dx, c[1]+dy)
                if not (x0 <= n[0] <= x1 and y0 <= n[1] <= y1):
                    continue
                e = extra.get(n, 0.0)
                if e == math.inf:
                    if (n[0]-s[0])**2+(n[1]-s[1])**2 > cerca_inicio:
                        continue
                    e = 2*COSTE_ZONA
                if n not in self.libres and dentro_de((n[0]*CELDA_MAPA, n[1]*CELDA_MAPA), zona_u):
                    e += COSTE_DESCONOCIDO
                if lado is not None and lado(n[0]*CELDA_MAPA, n[1]*CELDA_MAPA):
                    e += COSTE_OTRO_LADO
                nuevo = g_coste[c]+paso*(1+e)
                if nuevo < g_coste.get(n, math.inf):
                    g_coste[n], padre[n] = nuevo, c
                    heapq.heappush(abiertos, (nuevo+math.hypot(n[0]-g[0], n[1]-g[1]), n))
        if g not in padre:
            return None
        celdas = [g]
        while celdas[-1] != s:
            celdas.append(padre[celdas[-1]])
        puntos, ultimo = [], inicio
        for cx, cy in reversed(celdas[:-1]):
            q = (cx*CELDA_MAPA, cy*CELDA_MAPA)
            if math.hypot(q[0]-ultimo[0], q[1]-ultimo[1]) >= PASO_RUTA:
                puntos.append(q)
                ultimo = q
        return puntos+[meta]

    def dentro(self, punto, centro, radio=1.5):
        """El punto esta dentro de la envolvente convexa de lo ocupado a menos
        de radio del centro (el minimo local): dentro de la U."""
        return dentro_de(punto, self.envolvente(centro, radio))

    def envolvente(self, centro, radio):
        """Envolvente convexa (lista de vertices) de lo ocupado a menos de radio
        del centro; [] si hay menos de 3 puntos."""
        pts = sorted({(cx*CELDA_MAPA, cy*CELDA_MAPA) for cx, cy in self.ocupadas
                      if math.hypot(cx*CELDA_MAPA-centro[0], cy*CELDA_MAPA-centro[1]) <= radio})
        if len(pts) < 3:
            return []
        abajo, arriba = [], []
        for q in pts:
            while len(abajo) >= 2 and giro(abajo[-2], abajo[-1], q) <= 0:
                abajo.pop()
            abajo.append(q)
        for q in reversed(pts):
            while len(arriba) >= 2 and giro(arriba[-2], arriba[-1], q) <= 0:
                arriba.pop()
            arriba.append(q)
        envolvente = abajo[:-1]+arriba[:-1]
        return envolvente if len(envolvente) >= 3 else []


def giro(o, a, b):
    """Producto vectorial (a-o)x(b-o): >0 si o->a->b gira a la izquierda."""
    return (a[0]-o[0])*(b[1]-o[1])-(a[1]-o[1])*(b[0]-o[0])


def dentro_de(punto, envolvente):
    """El punto esta dentro del poligono convexo (vertices en sentido antihorario)."""
    return len(envolvente) >= 3 and all(
        giro(envolvente[i], envolvente[(i+1) % len(envolvente)], punto) >= 0
        for i in range(len(envolvente)))


# INTERFAZ ESPECIFICA DE P02
def validar_angulos(angulos):
    if len(angulos) != 7 or not all(math.isfinite(a) for a in angulos):
        raise ValueError("Introduzca siete angulos finitos en el orden real del SDK")
    if len(set(angulos)) != 7 or any(abs(a) > math.pi/2+1e-6 for a in angulos):
        raise ValueError("Los siete angulos deben ser distintos y estar entre -90 y 90 grados")




def cargar_calibracion(config):
    muestras = MUESTRAS_IR
    if config.calibracion_json:
        datos = json.loads(Path(config.calibracion_json).read_text(encoding="utf-8-sig"))
        if not isinstance(datos, dict):
            raise ValueError("El JSON debe contener {distancia_cm: [ir1,...,ir7]}")
        muestras = {float(d): [float(v) for v in fila] for d, fila in datos.items()}
    return CalibradorIR(muestras).validar_configuracion(config)


def editar_repulsion(c):
    print("1. Khatib: U=k/2*(1/d-1/d0)^2, crece con intensidad cerca del obstaculo.")
    print("2. Polinomico: U=k/2*(1-d/d0)^2, alternativa de crecimiento limitado.")
    print("Ambos actuan solo dentro de d0. Sus ganancias requieren ajustes diferentes.")
    while True:
        opcion = input(f"Funcion [actual: {c.repulsivo}; Intro conserva]: ").strip()
        if opcion in ("", "1", "2"):
            break
        print("Seleccione 1 o 2.")
    base = replace(c, repulsivo={"1": "khatib", "2": "polinomico"}.get(opcion, c.repulsivo))
    c = editar_parametros(base, (
        ("ganancia_repulsiva", "Ganancia repulsiva k", 1),
        ("influencia", "Distancia de influencia (cm; dentro del rango calibrado)", 100),
        ("fuerza_max_sensor", "Limite de fuerza por sensor", 1),
        ("liberar_esquiva", "Tiempo sin deteccion para volver a la meta (s)", 1),
        ("memoria_obstaculo", "Tiempo que se recuerda un obstaculo fuera de la vista (s)", 1)))
    while True:
        try:
            ktan = pedir_numero("Ganancia tangencial de rodeo (0 = repulsion pura)", c.ganancia_tangencial)
            return replace(c, ganancia_tangencial=ktan).validar()
        except ValueError as exc:
            print(exc)


def editar_sensores(c):
    alpha = pedir_numero("Filtro alpha (0 < alpha <= 1; 1 sin suavizado)", c.filtro_alpha, True)
    seguridad = pedir_numero("Distancia de parada (cm)", c.seguridad*100, True)/100
    giro = pedir_numero("IR frontal que provoca giro en el sitio (cm)", c.distancia_giro*100, True)/100
    v_cerca = pedir_numero("Avance maximo con obstaculo a la vista (cm/s)", c.v_cerca*100, True)/100
    print("JSON: {distancia_cm: [ir1,...,ir7]}; muestras medias de cada sensor.")
    ruta = input("Ruta JSON (Intro conserva; 'heredada' recupera la tabla de T03): ").strip().strip('"')
    ruta = c.calibracion_json if not ruta else "" if ruta.lower() == "heredada" else ruta
    try:
        nuevo = replace(c, filtro_alpha=alpha, seguridad=seguridad, distancia_giro=giro,
                        v_cerca=v_cerca, calibracion_json=ruta).validar()
        cargar_calibracion(nuevo)
        return nuevo
    except (ValueError, TypeError, OSError) as exc:
        print(f"No se guardaron los cambios de sensores: {exc}")
        return c


# LECTURAS Y DECISION: SIN ACCESO AL SDK.
@dataclass(frozen=True)
class Lectura:
    pose: Pose
    tiempo_pose: float  # Reloj monotono al recibir odometria e IR (lectura simultanea).
    intensidades: tuple
    distancias: tuple
    filtradas: tuple
    detecciones: tuple = (False,)*7
    presencia: tuple = (False,)*7  # Intensidad >= presencia_ir*umbral: algo se acerca.


@dataclass(frozen=True)
class Decision:
    estado: str
    distancia: float
    u_atr: float
    u_rep: float
    f_atr: tuple
    f_rep: tuple
    f_res: tuple
    comando: tuple
    evento: str = ""
    f_tan: tuple = (0.0, 0.0)
    lado: int = 0
    supervisor: str = "NAVEGACION"  # Estado del supervisor de minimos (P03).
    escapes: int = 0
    meta_activa: tuple = (math.nan, math.nan)  # Meta real o submeta virtual.


def limite_por_percepcion(distancias, config, dt):
    """Limite adicional por alcance IR y proximidad, independiente del filtro.

    No suponer espacio infinito si no hay deteccion: limitar al radio util.
    Reservar margen hasta seguridad y una latencia de DOS ciclos: la lectura
    llega con un ciclo de retraso y la orden actua otro ciclo despues
    (20261103_102514: obstaculo visto a 22 cm a 0.3 m/s, choque 0.8 s despues).
    Es un modelo conservador que requiere validacion fisica, no una garantia.
    """
    disponible = max(0.0, min(config.influencia, *distancias)-config.seguridad)
    bt = config.frenado*2*min(max(config.periodo, dt), RETRASO_MAX)
    return math.sqrt(bt*bt+2*config.frenado*disponible)-bt


MARGEN_GIRO = 0.03  # m de histeresis para dejar de girar y volver a avanzar.
ANGULO_FRONTAL = 0.5  # rad: sensores que miran al frente (0 y +-25 grados).
ANGULO_DELANTE = 1.0  # rad: sensores en la trayectoria (0, +-25 y +-50 grados).
RETRASO_MAX = 0.8  # s: un ciclo mas largo no se usa para predecir el giro.
LIMITE_LATERAL = 2.0  # Empuje lateral maximo, en veces la atraccion (simulacion).


def decidir(lectura, config, rampa, esquiva, dt, directo=False, lento=None):
    """Piloto P02. directo (P03): derecho a la meta (un punto de un camino ya
    planificado o recorrido), con el giro de emergencia mas cerca. lento
    (por defecto = directo): avance limitado a v_cerca."""
    lento = directo if lento is None else lento
    pose = lectura.pose
    dx, dy = config.meta_x-pose.x, config.meta_y-pose.y
    distancia = math.hypot(dx, dy)
    ua, fx, fy = atractivo(dx, dy, config)
    atr = a_local(fx, fy, pose.theta)
    # Repulsion desde la memoria (incluye lo que se ve ahora): no se anula
    # cuando el obstaculo pasa a la zona ciega lateral o trasera.
    esquiva.memoria.actualizar(pose, lectura.filtradas, lectura.tiempo_pose, config)
    distancias_mem, angulos_mem = esquiva.memoria.virtuales(pose)
    # Un punto a mas de distancia+RADIO_ROBOT del centro no se puede tocar de
    # camino a la meta (desigualdad triangular): se ignora. A 12 cm de la meta
    # el robot giraba sin parar por detecciones a 31-34 cm (20261103_114224).
    distancias_mem = [antes_de_meta(d, distancia) for d in distancias_mem]
    distancias = tuple(antes_de_meta(d, distancia) for d in lectura.distancias)
    ur, rx, ry, bx, by = repulsion(distancias_mem, config, angulos_mem)
    rep = (rx, ry)
    # Rodeo solo frente a lo que bloquea la trayectoria: con la repulsion
    # total y la memoria el lado no se liberaba nunca y el robot orbitaba.
    lado = esquiva.actualizar((bx, by), atr, dt, config)
    # Lo que bloquea no empuja: la atraccion, con su modulo, apunta al rumbo
    # que lo deja a un lado. Antes la repulsion (~5) y la atraccion (~0.3) se
    # peleaban y el robot rebotaba lejos del obstaculo antes de rodearlo.
    # Los puntos que bloquean siguen limitando la velocidad y el giro en el sitio.
    rumbo = desvio(distancias_mem, angulos_mem, lado, config) if lado else None
    guia = atr
    if rumbo is not None:
        meta_local = math.atan2(atr[1], atr[0])
        if lado*(rumbo-meta_local) > 0:
            modulo = math.hypot(*atr)
            guia = (modulo*math.cos(rumbo), modulo*math.sin(rumbo))
    # Empuje lateral y rodeo de lo que queda a los lados, como mucho
    # LIMITE_LATERAL veces la atraccion: apartarse sin salir despedido.
    tan = tangencial(distancias_mem, angulos_mem, config)
    lateral = (rx-bx+tan[0], ry-by+tan[1])
    tope = LIMITE_LATERAL*math.hypot(*atr)
    if math.hypot(*lateral) > tope:
        escala = tope/math.hypot(*lateral)
        lateral = (lateral[0]*escala, lateral[1]*escala)
        tan = (tan[0]*escala, tan[1]*escala)
    res = (guia[0]+lateral[0], guia[1]+lateral[1])
    if directo:
        # Camino del mapa o rastro ya recorrido: derecho al punto. Cerca de un
        # punto la atraccion es debil y el empuje de lo recordado lo hacia
        # orbitar 27 s sin alcanzarlo (20261103_163541). Seguridad intacta.
        # Solo la direccion: el modulo de la atraccion a un punto a 20 cm es
        # ~0.05 y la velocidad (kv*|F|) subia y bajaba en cada punto: iba a
        # tirones entre 6 y 12 cm/s (20261103_172059).
        modulo = math.hypot(*atr) or 1.0
        res = (atr[0]/modulo, atr[1]/modulo)
    # Obstaculo delante o demasiado cerca: no abortar, no quedarse parado y no
    # confiar solo en el equilibrio de fuerzas (frente a un obstaculo centrado
    # la repulsion apenas gira). El chasis es circular: girar en el sitio no
    # acerca el contorno al obstaculo. Distancias sin filtrar: sin retraso.
    frontales = [i for i, a in enumerate(config.angulos_ir) if abs(a) < ANGULO_FRONTAL]
    delante = [i for i, a in enumerate(config.angulos_ir) if abs(a) < ANGULO_DELANTE]
    frontal = min(distancias[i] for i in frontales)
    # Siguiendo un camino se va cerca de las paredes: los IR laterales
    # (+-75 grados) no cuentan, o el giro seguia hasta perder el camino
    # (20261103_160033), y el giro en el sitio queda como emergencia, mas
    # cerca y mas lento (a 12 cm chocaba en un rincon con 2 ciclos de retraso).
    cercania = min(distancias[i] for i in delante) if directo else min(distancias)
    giro = min(config.distancia_giro, GIRO_ESCAPE) if directo else config.distancia_giro
    wmax = WMAX_ESCAPE if directo else config.wmax
    if cercania <= config.seguridad or frontal <= giro:
        esquiva.girando = True
    elif cercania > config.seguridad+MARGEN_GIRO and frontal > giro+MARGEN_GIRO:
        esquiva.girando = False
    if esquiva.girando and distancia > config.tolerancia:
        if not esquiva.lado:
            esquiva.lado = elegir_lado(repulsion(distancias, config)[1:3], atr)
        esquiva.libre = 0.0
        rampa.actual = 0.0
        # Siguiendo un camino, girar hacia el punto: girar hacia el lado de
        # esquiva mientras el camino tiraba al contrario dejo al robot 15 s
        # oscilando frente a la pared (20261103_165141).
        sentido = (1 if atr[1] >= 0 else -1) if directo else esquiva.lado
        comando = ruedas(0, sentido*wmax, config)
        rampa.giro = comando[3]
        return Decision("GIRO_OBSTACULO", distancia, ua, ur, atr, rep, res, comando,
                        "Obstaculo delante: giro en el sitio", tan, esquiva.lado)
    if distancia <= config.tolerancia:
        rampa.actual = 0.0
        error = 0 if config.meta_theta is None else angulo_corto(config.meta_theta-pose.theta)
        estado = "LLEGADA" if abs(error) <= config.tolerancia_angular else "ORIENTACION_FINAL"
        w = 0 if estado == "LLEGADA" else max(-config.wmax, min(config.wmax, config.kw*error))
        comando = ruedas(0, w, config)
        rampa.giro = comando[3]
        return Decision(estado, distancia, ua, ur, atr, rep, res, comando, "", tan, lado)
    # Misma ley de P01, ahora con F_atr + F_rep + F_tan en el marco local.
    # Retraso estimado = ultimo ciclo; acotado ante cortes Bluetooth.
    retraso = min(max(dt, config.periodo), RETRASO_MAX)
    v, w = velocidades(*res, distancia, config, rampa.giro, retraso)
    w = max(-wmax, min(wmax, w))
    # Solo frenan los obstaculos en la trayectoria (+-50 grados): uno al costado
    # o recordado detras mientras se rodea no se acerca al avanzar. Antes
    # cualquier deteccion imponia v_cerca y el 77% del recorrido iba a 5 cm/s.
    v = min(v, limite_por_percepcion([distancias[i] for i in delante], config, dt))
    # Con un obstaculo delante avanzar despacio: entre lectura y orden pasan
    # ~2 ciclos. La presencia (intensidad por debajo del umbral calibrado)
    # solo en los frontales: el IR 6 (-50 grados) tiene umbral 15.1 y su ruido
    # llega a 15; frenaba a v_cerca hasta el 40% del tiempo sin obstaculos
    # (20261103_113443 y 113635).
    # La presencia esta mas alla del alcance calibrado: tampoco cuenta si ese
    # alcance ya queda detras de la meta.
    presencia = math.isfinite(antes_de_meta(config.influencia, distancia))
    # En el escape, siempre despacio: con el giro limitado a WMAX_ESCAPE el
    # radio de curva es v/w; a 0.21 m/s abria la esquina del brazo de la U
    # (simulacion con 2 ciclos de retraso).
    if lento or any(math.isfinite(distancias[i]) for i in delante) or (presencia and any(
            lectura.presencia[i] for i in frontales)):
        v = min(v, config.v_cerca)
    v = rampa.limitar(v, dt, config)
    comando = ruedas(v, w, config)
    rampa.actual, rampa.giro = comando[2], comando[3]
    estado = "ESQUIVA" if lado else "NAVEGACION"
    return Decision(estado, distancia, ua, ur, atr, rep, res, comando, "", tan, lado)


# SUPERVISOR DE MINIMOS LOCALES (PARTE 03).
class DetectorMinimo:
    """Minimo local = en ventana_minimo segundos la distancia a la meta no
    mejora progreso_minimo respecto al inicio de la ventana.

    Cubre la parada (F_res ~ 0), la oscilacion y el giro en el sitio frente al
    fondo de una U: en los tres casos el robot no se acerca a la meta. No se
    exige F_res = 0 exacto porque el robot real oscila alrededor del minimo.
    """
    def __init__(self):
        self.historial = deque()

    def reiniciar(self):
        self.historial.clear()

    def actualizar(self, t, distancia, config):
        self.historial.append((t, distancia))
        while len(self.historial) > 1 and self.historial[1][0] <= t-config.ventana_minimo:
            self.historial.popleft()
        t0, d0 = self.historial[0]
        if t-t0 < config.ventana_minimo or distancia <= config.tolerancia:
            return False
        return d0-min(d for _, d in self.historial) < config.progreso_minimo


TOLERANCIA_SUBMETA = 0.15  # m: la submeta solo orienta, no exige precision.


class Supervisor:
    """Maquina de estados: NAVEGACION -> ESCAPE_CONTORNO | ESCAPE_SUBMETA
    (-> CAMINO_MAPA) -> NAVEGACION.

    El piloto P02 (decidir) sigue actuando en todos los estados, con su
    seguridad: el supervisor solo cambia la meta (puntos del camino del mapa
    o submetas). Cada nuevo intento de submeta usa el lado contrario.
    """
    def __init__(self, config):
        self.config = config
        self.detector = DetectorMinimo()
        self.estado = "NAVEGACION"
        self.escapes = self.detectados = self.fallidos = 0
        self.rodeo = 0  # +1 rodear por la izquierda, -1 por la derecha.
        self.inicio = self.distancia_inicio = 0.0
        self.submeta = None
        self.mapa = MapaTemporal()
        self.minimo = (0.0, 0.0)  # Donde se detecto el ultimo minimo local.
        self.dentro = False  # Robot dentro de la U (envolvente del mapa).
        self.t_plan, self.version_plan = 0.0, -1
        self.lado_camino = 0  # Lado (+1 izquierda) del primer camino del mapa.
        self.pendientes = []  # Submetas que faltan tras la actual.
        self.extras = []  # Submetas lateral y de avance (fuera de la U).
        self.trayectoria = deque(maxlen=400)  # Puntos recorridos cada PASO_TRAYECTORIA.

    def pilotar(self, lectura, rampa, esquiva, t, dt):
        c, pose = self.config, lectura.pose
        distancia = math.hypot(c.meta_x-pose.x, c.meta_y-pose.y)
        evento = self.comprobar_fin(pose, distancia, esquiva, t) if self.estado != "NAVEGACION" else ""
        if not self.trayectoria or math.hypot(pose.x-self.trayectoria[-1][0],
                                              pose.y-self.trayectoria[-1][1]) >= PASO_TRAYECTORIA:
            self.trayectoria.append((pose.x, pose.y))
        self.mapa.actualizar(pose, lectura.filtradas, c.angulos_ir)
        if self.estado in ("ESCAPE_CONTORNO", "CAMINO_MAPA"):
            # Camino del mapa; se rehace si los IR ven paredes nuevas.
            if self.mapa.version != self.version_plan and t-self.t_plan >= REPLANIFICAR:
                self.planificar(pose, t)
            dentro = self.dentro
            # Rodeado de paredes a RADIO_CONCAVIDAD del robot (como el detector).
            # Con la envolvente alrededor del minimo, los demas obstaculos de la
            # zona la agrandaban y el robot seguia "dentro" y lento lejos de la
            # U (20261103_171631).
            # Una vez fuera, fuera: al pasar de nuevo junto a la boca volvia a
            # ponerse amarillo (20261103_173601).
            self.dentro = self.dentro and self.mapa.dentro((pose.x, pose.y), (pose.x, pose.y),
                                                           RADIO_CONCAVIDAD)
            decision = self.hacia_submetas(lectura, rampa, esquiva, dt)
            if decision is None and distancia > FINAL_P02+TOLERANCIA_SUBMETA:
                # Camino agotado lejos de la meta (era la ruta de reserva por
                # el rastro): otro camino, no volver al piloto P02.
                self.planificar(pose, t)
                decision = self.hacia_submetas(lectura, rampa, esquiva, dt)
            if decision is None:
                evento = self.terminar("Camino del mapa completado")
                decision = decidir(lectura, c, rampa, esquiva, dt)
            elif dentro and not self.dentro:
                evento = "Fuera de la U: rodeandola por el mapa"
        elif self.estado == "ESCAPE_SUBMETA":
            saliendo = self.saliendo
            decision = self.hacia_submetas(lectura, rampa, esquiva, dt)
            if decision is None:
                # Hasta la meta por el camino del mapa, como el contorno: con
                # el piloto P02 volvia a esquivar detecciones sueltas y a
                # disparar el detector de camino a la meta (20261103_170319).
                self.estado = "CAMINO_MAPA"
                self.planificar(pose, t)
                evento = "Submetas alcanzadas: camino del mapa hasta la meta"
                decision = self.hacia_submetas(lectura, rampa, esquiva, dt)
                if decision is None:
                    evento = self.terminar("Camino del mapa completado")
                    decision = decidir(lectura, c, rampa, esquiva, dt)
            elif saliendo and not self.saliendo:
                evento = (f"Fuera de la U: submeta lateral "
                          f"({self.submeta[0]*100:.0f}, {self.submeta[1]*100:.0f}) cm")
        else:
            decision = decidir(lectura, c, rampa, esquiva, dt)
            if (decision.estado not in ("LLEGADA", "ORIENTACION_FINAL")
                    and self.detector.actualizar(t, distancia, c)):
                # Sin progreso no basta: rodeando un objeto suelto el robot
                # tambien se aleja un rato (20261103_165849, 165652). En un
                # minimo local (U) esta rodeado de paredes: dentro de la
                # envolvente del mapa cercano. Con un objeto convexo, nunca.
                if self.mapa.dentro((pose.x, pose.y), (pose.x, pose.y), RADIO_CONCAVIDAD):
                    evento = self.minimo_detectado(pose, distancia, esquiva, t)
                else:
                    self.detector.reiniciar()
                    evento = "Sin progreso, pero sin paredes alrededor: no es minimo local"
        meta = self.submeta if self.estado != "NAVEGACION" and self.submeta else (c.meta_x, c.meta_y)
        return replace(decision, distancia=distancia, supervisor=self.estado, escapes=self.escapes,
                       meta_activa=meta, evento="; ".join(e for e in (decision.evento, evento) if e))

    def minimo_detectado(self, pose, distancia, esquiva, t):
        c = self.config
        self.detectados += 1
        self.detector.reiniciar()
        if c.estrategia_escape == "ninguna":
            return "Minimo local detectado (recuperacion desactivada)"
        if self.escapes >= c.max_escapes:
            return "Minimo local detectado: limite de escapes alcanzado"
        self.escapes += 1
        self.inicio, self.distancia_inicio = t, distancia
        self.minimo = (pose.x, pose.y)
        self.lado_camino = 0
        if c.estrategia_escape == "contorno":
            self.estado = "ESCAPE_CONTORNO"
            self.dentro = self.mapa.dentro(self.minimo, self.minimo, RADIO_CONCAVIDAD)
            n = self.planificar(pose, t)
            return (f"Minimo local: escape {self.escapes} por contorno con el mapa "
                    f"({len(self.mapa.ocupadas)} celdas ocupadas, camino de {n} puntos)")
        # Submetas: salir de la U deshaciendo el rastro (ruta_de_salida). Ir
        # en linea recta a la salida cortaba la esquina de la U (20261103_162735).
        ruta = self.ruta_de_salida(pose)
        retro = ruta[-1]
        # Tramo 2: a un lado, perpendicular a la meta, fuera del ancho de la U.
        # Tramo 3: en paralelo a la U hasta dejarla atras. Desde la lateral
        # derecho a la meta se cruzaba la punta del brazo (simulacion).
        # Lado y distancias segun lo que el mapa sabe de la U: el lado salia
        # del giro del momento y una vez rodeo por donde habia mas obstaculos
        # (20261103_165141). Despues de un intento fallido, el otro lado.
        meta = math.atan2(c.meta_y-retro[1], c.meta_x-retro[0])
        izquierda, derecha = self.medir_u(retro, meta)
        if self.rodeo:
            self.rodeo = -self.rodeo
        elif abs(izquierda-derecha) > CELDA_MAPA*2:
            self.rodeo = 1 if izquierda < derecha else -1
        else:
            self.rodeo = esquiva.lado or 1
        # Lo justo para librar la U; despues el camino del mapa la rodea (con
        # 1 m fijo y otro punto de avance el robot se iba lejisimos: 171356).
        aparte = max(c.distancia_submeta, (izquierda if self.rodeo > 0 else derecha)+HOLGURA_SUBMETA)
        lateral = (retro[0]+aparte*math.cos(meta+self.rodeo*math.pi/2),
                   retro[1]+aparte*math.sin(meta+self.rodeo*math.pi/2))
        self.extras = [lateral]
        self.submeta, self.pendientes = ruta[0], ruta[1:]+self.extras
        self.estado = "ESCAPE_SUBMETA"
        return (f"Minimo local: escape {self.escapes} por submetas: retroceso a "
                f"({retro[0]*100:.0f}, {retro[1]*100:.0f}) cm y lateral a "
                f"({lateral[0]*100:.0f}, {lateral[1]*100:.0f}) cm")

    def comprobar_fin(self, pose, distancia, esquiva, t):
        c = self.config
        if distancia <= c.tolerancia:
            return self.terminar("Meta alcanzada durante el escape")
        if t-self.inicio >= c.tiempo_max_escape:
            self.fallidos += 1
            return self.terminar("Escape agotado sin salir: el siguiente intento cambia de lado")
        # El contorno no vuelve al piloto P02 a mitad de camino: P02 no usa el
        # mapa y volvia a meterse en la U (20261103_165429). Sigue el camino
        # del mapa hasta la meta.
        return ""

    def hacia_submetas(self, lectura, rampa, esquiva, dt):
        """Piloto P02 hacia la submeta actual; alcanzada, a la siguiente.

        None cuando no quedan. Nunca devuelve LLEGADA: la mision acababa en
        un punto del rastro como si fuera la meta (simulacion).
        """
        rampa_antes = rampa.actual, rampa.giro
        while self.submeta is not None:
            sub = replace(self.config, meta_x=self.submeta[0], meta_y=self.submeta[1],
                          meta_theta=None, tolerancia=TOLERANCIA_SUBMETA)
            contorno = self.estado in ("ESCAPE_CONTORNO", "CAMINO_MAPA")
            # Todo el escape va derecho a sus puntos (seguridad frontal intacta):
            # hacia la submeta lateral, el piloto P02 lo desviaba por lo que
            # recordaba y se alejaba de mas (20261103_171356).
            decision = decidir(lectura, sub, rampa, esquiva, dt, directo=True,
                               lento=self.dentro if contorno else self.saliendo)
            if decision.estado != "LLEGADA":
                return decision
            # Un punto intermedio no es parar: LLEGADA ponia la rampa a cero y
            # el robot arrancaba de nuevo cada 20 cm a 3 cm/s (20261103_170106).
            rampa.actual, rampa.giro = rampa_antes
            self.submeta = self.pendientes.pop(0) if self.pendientes else None
        return None

    @property
    def saliendo(self):
        """Saliendo de la U (luz amarilla): en el contorno, dentro de la
        envolvente del mapa; con submetas, recorriendo el rastro."""
        if self.estado == "ESCAPE_CONTORNO":
            return self.dentro
        return (self.estado == "ESCAPE_SUBMETA" and self.submeta is not None
                and self.submeta not in self.extras)

    def ruta_de_salida(self, pose):
        """Camino de ENTRADA deshecho: puntos del rastro cada PASO_RUTA, desde
        la primera vez que el robot paso cerca de aqui hacia atras, hasta uno
        a RETROCESO o mas de aqui en linea recta.

        Ir en linea recta a ese punto cortaba la esquina de la U y el robot
        choco con el brazo (20261103_162735): el rastro en cambio esta libre.
        Empezar por la primera pasada deja fuera las vueltas dentro de la U;
        recorrerlas otra vez devolvia el robot al fondo (20261103_163114).
        """
        puntos = list(self.trayectoria)
        lejos = [i for i, q in enumerate(puntos) if math.hypot(q[0]-pose.x, q[1]-pose.y) >= RETROCESO]
        inicio = lejos[-1] if lejos else 0
        entrada = next((i for i in range(inicio, len(puntos))
                        if math.hypot(puntos[i][0]-pose.x, puntos[i][1]-pose.y) <= CERCA_RASTRO),
                       len(puntos)-1)
        ruta, ultimo = [], (pose.x, pose.y)
        for q in reversed(puntos[inicio:entrada+1]):
            if math.hypot(q[0]-ultimo[0], q[1]-ultimo[1]) >= PASO_RUTA:
                ruta.append(q)
                ultimo = q
        if puntos and (not ruta or ruta[-1] != puntos[inicio]):
            ruta.append(puntos[inicio])
        return ruta or [(pose.x, pose.y)]

    def medir_u(self, origen, rumbo):
        """Cuanto sobresale la U (lo ocupado a menos de RADIO_U del minimo) a
        la izquierda y a la derecha de la recta origen-rumbo."""
        ux, uy = math.cos(rumbo), math.sin(rumbo)
        izquierda = derecha = 0.0
        for cx, cy in self.mapa.ocupadas:
            x, y = cx*CELDA_MAPA, cy*CELDA_MAPA
            if math.hypot(x-self.minimo[0], y-self.minimo[1]) > RADIO_U:
                continue
            lado = (x-origen[0])*-uy+(y-origen[1])*ux
            izquierda, derecha = max(izquierda, lado), max(derecha, -lado)
        return izquierda, derecha

    def lateral(self, q):
        """Distancia con signo (+ izquierda) de q a la recta minimo -> meta."""
        c = self.config
        ux, uy = c.meta_x-self.minimo[0], c.meta_y-self.minimo[1]
        n = math.hypot(ux, uy) or 1.0
        return ((q[0]-self.minimo[0])*-uy+(q[1]-self.minimo[1])*ux)/n

    def otro_lado(self):
        """Funcion (x, y) -> True en el lado contrario al del primer camino,
        cerca de la U: alli el camino cuesta mas y no cambia de lado por un
        obstaculo nuevo (20261103_173601: salio por el oeste, vio otro objeto
        y cruzo por delante de la U al este). None antes del primer camino."""
        if not self.lado_camino:
            return None
        return lambda x, y: (math.hypot(x-self.minimo[0], y-self.minimo[1]) <= ZONA_LADO
                             and self.lado_camino*self.lateral((x, y)) < -RADIO_ROBOT)

    def planificar(self, pose, t):
        """Camino del mapa hasta la meta con holgura = radio IR + distancia_pared
        (los IR verian la pared a distancia_pared). Sin camino, el rastro."""
        c = self.config
        camino = None
        for holgura in (RADIO_IR+c.distancia_pared, RADIO_IR+c.distancia_pared-0.04, RADIO_ROBOT+0.03):
            camino = self.mapa.camino((pose.x, pose.y), (c.meta_x, c.meta_y), holgura,
                                      self.mapa.envolvente(self.minimo, RADIO_U), self.otro_lado())
            if camino is not None:
                break
        if camino is not None:
            # El ultimo tramo, con el piloto P02, que frena al llegar: siguiendo
            # el camino hasta 15 cm de la meta llegaba lanzado y daba una vuelta.
            camino = [q for q in camino if math.hypot(q[0]-c.meta_x, q[1]-c.meta_y) > FINAL_P02]
        if camino is None and self.submeta is None:
            camino = self.ruta_de_salida(pose)
        if camino and not self.lado_camino:
            # Primer camino: su lado de la recta minimo-meta queda elegido.
            self.lado_camino = 1 if sum(self.lateral(q) for q in camino) > 0 else -1
        if camino:
            self.submeta, self.pendientes = camino[0], camino[1:]
        elif camino is not None:
            self.submeta, self.pendientes = None, []  # Ya en el tramo final.
        self.t_plan, self.version_plan = t, self.mapa.version
        return len(camino) if camino else 0

    def terminar(self, motivo):
        self.estado = "NAVEGACION"
        self.submeta, self.dentro = None, False
        self.detector.reiniciar()
        return motivo


RETROCESO = 0.8  # m: distancia del punto de la trayectoria al que se retrocede.
REPLANIFICAR = 1.0  # s minimos entre dos caminos del mapa (si el mapa cambia).
COSTE_OTRO_LADO = 4.0  # Coste extra por celda en el otro lado de la U (compromiso de lado).
ZONA_LADO = 2.0  # m alrededor del minimo en que cuenta el compromiso de lado.
RADIO_CONCAVIDAD = 1.0  # m: paredes que cuentan para decidir si el robot esta rodeado.
FINAL_P02 = 0.4  # m: el camino del mapa acaba aqui; la llegada la hace el piloto P02.
PASO_TRAYECTORIA = 0.05  # m entre puntos guardados de la trayectoria.
PASO_RUTA = 0.2  # m entre puntos de la ruta de salida por el rastro.
HOLGURA_SUBMETA = 0.45  # m: radio del robot + margen sobre lo que sobresale la U.
RADIO_U = 1.5  # m alrededor del minimo en que lo ocupado se considera la U.
CERCA_RASTRO = 0.25  # m: primera pasada por aqui = punto donde empieza la ruta de salida.


class Registro:
    CAMPOS = ("t_s", "Ts_s", "dt_s", "x_m", "y_m", "theta_rad", "distancia_meta_m",
              "U_atr", "U_rep", "F_atr_x", "F_atr_y", "F_rep_x", "F_rep_y",
              "F_res_x", "F_res_y", "F_res_modulo", "F_res_angulo_rad",
              "v_m_s", "w_rad_s", "v_izq_m_s", "v_der_m_s", "estado", "evento",
              "retraso_s", "v_odom_m_s", "w_odom_rad_s", "F_tan_x", "F_tan_y", "lado_esquiva",
              "supervisor", "escapes", "meta_activa_x_m", "meta_activa_y_m") + tuple(
                  f"{prefijo}{i}" for prefijo in ("ir_", "d_m_", "d_filtrada_m_", "deteccion_") for i in range(1, 8))

    def __init__(self, config, calibrador):
        self.carpeta = Path(__file__).resolve().parent / "datos" / Path(__file__).stem / datetime.now().strftime("%Y%m%d_%H%M%S_%f")
        self.carpeta.mkdir(parents=True)
        for nombre, datos in (("configuracion", asdict(config)), ("calibracion", calibrador.muestras)):
            (self.carpeta/f"{nombre}.json").write_text(json.dumps(datos, indent=2), encoding="utf-8")
        self.archivo = (self.carpeta/"recorrido.csv").open("w", newline="", encoding="utf-8")
        self.csv = csv.DictWriter(self.archivo, fieldnames=self.CAMPOS)
        self.csv.writeheader()
        self.minima, self.vmax, self.ciclos_tarde = math.inf, 0.0, 0
        self.plazos_omitidos = 0
        self.muestra_anterior = None
        self.vmax_odom = None

    def ciclo(self, t, dt, lectura, decision, retraso, config):
        d, p = decision, lectura.pose
        izq, der, v, w = d.comando
        self.minima = min(self.minima, *lectura.distancias)
        self.vmax = max(self.vmax, abs(v))
        self.ciclos_tarde += int(dt > config.periodo*1.05)
        vo = wo = None
        if self.muestra_anterior is not None:
            t0, p0 = self.muestra_anterior
            intervalo = lectura.tiempo_pose-t0
            if intervalo > 0:
                vo = math.hypot(p.x-p0.x, p.y-p0.y)/intervalo
                wo = angulo_corto(p.theta-p0.theta)/intervalo
                self.vmax_odom = max(self.vmax_odom or 0, vo)
        self.muestra_anterior = (lectura.tiempo_pose, p)
        valores = (t, config.periodo, dt, p.x, p.y, p.theta, d.distancia, d.u_atr, d.u_rep,
                   *d.f_atr, *d.f_rep, *d.f_res, math.hypot(*d.f_res), math.atan2(d.f_res[1], d.f_res[0]),
                   v, w, izq, der, d.estado, d.evento, retraso, vo, wo, *d.f_tan, d.lado,
                   d.supervisor, d.escapes, *d.meta_activa,
                   *lectura.intensidades,
                   *(v if math.isfinite(v) else "" for v in lectura.distancias),
                   *(v if math.isfinite(v) else "" for v in lectura.filtradas),
                   *(int(activa) for activa in lectura.detecciones))
        self.csv.writerow(dict(zip(self.CAMPOS, valores)))
        self.archivo.flush()

    def evento(self, t, texto):
        self.csv.writerow({"t_s": t, "estado": "FIN", "evento": texto})
        self.archivo.flush()

    def cerrar(self, resultado, tiempo, error, colision, supervisor):
        self.archivo.close()
        datos = {"resultado": resultado, "exito": resultado == "meta_alcanzada" and not colision,
                 "tiempo_s": tiempo, "error_final_m": error, "colision": colision,
                 "estrategia_escape": supervisor.config.estrategia_escape,
                 "minimos_detectados": supervisor.detectados, "escapes": supervisor.escapes,
                 "escapes_agotados": supervisor.fallidos,
                 "distancia_minima_ir_m": self.minima if math.isfinite(self.minima) else None,
                 "vmax_ordenada_m_s": self.vmax, "vmax_estimada_odometria_m_s": self.vmax_odom,
                 "ciclos_lentos": self.ciclos_tarde, "plazos_omitidos": self.plazos_omitidos,
                 "nota": "Minima IR estimada, no separacion de todo el chasis. Velocidad odometrica estimada; ciclo lento dt > 1.05*Ts."}
        (self.carpeta/"resultado.json").write_text(json.dumps(datos, indent=2), encoding="utf-8")
        # Mapa temporal (celdas ocupadas) para dibujar la U y el recorrido.
        with open(self.carpeta/"mapa.csv", "w", newline="", encoding="utf-8") as archivo:
            escritor = csv.writer(archivo)
            escritor.writerow(("x_m", "y_m"))
            escritor.writerows((round(cx*CELDA_MAPA, 3), round(cy*CELDA_MAPA, 3))
                               for cx, cy in sorted(supervisor.mapa.ocupadas))


async def navegar(adaptador, config, calibrador):
    registro = Registro(config, calibrador)
    rampa = RampaAvance()
    esquiva = MemoriaEsquiva()
    supervisor = Supervisor(config)
    inicio = anterior = plazo = time.monotonic()
    resultado, error = "interrumpida", None
    print(f"Registro: {registro.carpeta}")
    try:
        while True:
            adaptador.comprobar_seguridad()
            if time.monotonic()-inicio >= config.tiempo_max:
                raise TimeoutError("Tiempo maximo de mision alcanzado")
            lectura = await adaptador.leer_entorno()
            ahora = time.monotonic()
            if ahora-inicio >= config.tiempo_max:
                raise TimeoutError("Tiempo maximo de mision alcanzado durante la lectura")
            decision = supervisor.pilotar(lectura, rampa, esquiva, ahora-inicio, ahora-anterior)
            error = decision.distancia
            if "Minimo local" in decision.evento:
                print(f"{ahora-inicio:6.1f} s  {decision.evento}")
            await adaptador.aplicar(*decision.comando[:2])
            await adaptador.luz(AMARILLO if supervisor.saliendo else AZUL)
            registro.ciclo(ahora-inicio, ahora-anterior, lectura, decision, max(0, ahora-plazo), config)
            if decision.estado == "LLEGADA":
                resultado = "meta_alcanzada"
                break
            anterior = ahora
            plazo += config.periodo
            actual = time.monotonic()
            if plazo < actual:
                omitidos = math.floor((actual-plazo)/config.periodo)+1
                registro.plazos_omitidos += omitidos
                plazo += omitidos*config.periodo
            await asyncio.sleep(max(0, plazo-time.monotonic()))
    except BaseException as exc:
        resultado = f"{type(exc).__name__}: {exc}"
        raise
    finally:
        try:
            await adaptador.parar()
        finally:
            if adaptador.colision:
                resultado = "Parachoques activado; mision cancelada"
            try:
                registro.evento(time.monotonic()-inicio, resultado)
            finally:
                registro.cerrar(resultado, time.monotonic()-inicio, error, adaptador.colision, supervisor)
    return resultado


# ADAPTADOR: LAS UNIDADES DEL SDK SE CONVIERTEN SOLO EN ESTE LIMITE.
AZUL, AMARILLO = (0, 0, 255), (255, 200, 0)  # Navegando / saliendo de un minimo local.
INTENTOS_PRIMERA_LECTURA = 3  # Solo con el robot parado, antes de la primera pose.
INTENTOS_LECTURA = 3  # En marcha: parar y reintentar antes de abortar (20261103_160033).


class Adaptador:
    def __init__(self, robot, config, calibrador):
        self.robot, self.config, self.calibrador = robot, config, calibrador
        self.filtro = FiltroIR(config.filtro_alpha)
        self.detector = DetectorIR(calibrador, config)
        self.colision = False
        self.origen_sdk = None
        self.color = AZUL  # Puesto en iniciar() antes de navegar.

    def comprobar_seguridad(self):
        if self.colision:
            raise RuntimeError("Parachoques activado; mision cancelada")

    async def leer_entorno(self):
        self.comprobar_seguridad()
        # Peticiones simultaneas: el SDK empareja cada respuesta por su id.
        # En serie el ciclo real era ~0.4 s, demasiado para reaccionar con IR de 25 cm.
        # La primera lectura (robot parado) se reintenta: en 20261103_103743 y
        # 103821 no respondio en 2 s y la mision acababa sin ningun ciclo.
        # En marcha, un corte de 2 s abortaba la mision con el robot ya fuera
        # de la U (20261103_160033): ahora se para y se reintenta.
        intentos = INTENTOS_PRIMERA_LECTURA if self.origen_sdk is None else INTENTOS_LECTURA
        p = ir = None
        for intento in range(1, intentos+1):
            try:
                p, ir = await asyncio.wait_for(
                    asyncio.gather(self.robot.get_position(), self.robot.get_ir_proximity()),
                    self.config.timeout_lectura)
                break
            except (asyncio.TimeoutError, ValueError):
                # ValueError "coroutine already executing": el SDK tras un
                # corte a mitad de una orden (20261103_165652).
                if intento == intentos:
                    raise TimeoutError(
                        f"El robot no respondio a la lectura de odometria e IR en "
                        f"{self.config.timeout_lectura:g} s ({intentos} intento/s): revise el Bluetooth") from None
                print(f"Lectura sin respuesta; reintento {intento}/{intentos-1}")
                if self.origen_sdk is not None:
                    try:
                        await self.parar()  # No seguir a ciegas con la ultima orden.
                    except (asyncio.TimeoutError, ValueError):
                        pass
        if p is None or not all(math.isfinite(v) for v in (p.x, p.y, p.heading)):
            raise RuntimeError("Odometria no disponible")
        tiempo_pose = time.monotonic()
        pose_sdk = Pose(p.x/100, p.y/100, math.radians(p.heading))
        if self.origen_sdk is None:
            self.origen_sdk = pose_sdk  # Primera lectura parado despues del reinicio.
        pose = pose_en_escenario(pose_sdk, self.origen_sdk, self.config.orientacion_inicial)
        if ir is None or not hasattr(ir, "sensors"):
            raise RuntimeError("Sensores IR no disponibles")
        valores = tuple(ir.sensors)
        distancias = self.calibrador.convertir(valores)
        confirmadas = self.detector.actualizar(valores, distancias)
        self.comprobar_seguridad()
        # Distancias originales conservadas para parada inmediata y limite de avance.
        # Solo la repulsion recibe detecciones confirmadas y filtradas.
        presencia = tuple(v >= self.config.presencia_ir*u for v, u in zip(valores, self.detector.umbrales))
        return Lectura(pose, tiempo_pose, valores, distancias,
                       self.filtro.actualizar(confirmadas), tuple(self.detector.activas), presencia)

    async def aplicar(self, izq, der):
        self.comprobar_seguridad()
        try:
            await asyncio.wait_for(self.robot.set_wheel_speeds(100*izq, 100*der), self.config.timeout_lectura)
        except (asyncio.TimeoutError, ValueError):
            # Un corte Bluetooth puntual: un reintento antes de abortar.
            await asyncio.wait_for(self.robot.set_wheel_speeds(100*izq, 100*der), self.config.timeout_lectura)
        if self.colision:
            await self.parar()
            self.comprobar_seguridad()

    async def parar(self):
        await asyncio.wait_for(self.robot.set_wheel_speeds(0, 0), self.config.timeout_lectura)

    async def luz(self, rgb):
        """Cambia el anillo de luz solo si cambia el color (una orden Bluetooth)."""
        if rgb == self.color:
            return
        try:
            await asyncio.wait_for(self.robot.set_lights_on_rgb(*rgb), self.config.timeout_lectura)
            self.color = rgb
        except (asyncio.TimeoutError, ValueError):
            pass  # La luz no debe parar la mision; se reintenta en el ciclo siguiente.


def ejecutar(config, calibrador):
    from irobot_edu_sdk.backend.bluetooth import Bluetooth
    from irobot_edu_sdk.robots import Create3, event
    config.validar()
    calibrador.validar_configuracion(config)
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
            # Abrir registro antes de la primera lectura: registrar tambien fallos iniciales.
            await r.set_lights_on_rgb(*AZUL)
            resultado = await navegar(adaptador, config, calibrador)
            print(resultado)
            if resultado == "meta_alcanzada":
                await r.set_lights_on_rgb(255, 80, 0)
            else:
                await r.set_lights_blink_rgb(255, 0, 0)
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

# VALIDACION FISICA P02 (guion 4.2): no sustituir por datos inventados.
# Comprobar orden/angulos IR, ruido en reposo, intensidades medias y alcance util.
# Escenario                 | Funcion | k_rep/k_atr | Colision | d_min | Tiempo | Observaciones
# A. Un obstaculo           |         |             |          |       |        |
# B. Dos o mas obstaculos   |         |             |          |       |        |
# C. Paso estrecho          |         |             |          |       |        |
# D. Repulsion alternativa  |         |             |          |       |        |
# Ajustar ganancias segun sus unidades. Conservar CSV, configuracion y calibracion.
# P02 no incorpora recuperacion de minimos locales: corresponde a P03.
