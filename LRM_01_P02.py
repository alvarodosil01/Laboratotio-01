"""Parte 02: potencial atractivo + repulsivo con los 7 IR. Grupo 4.

Parte de LRM_01_P01.py (misma interfaz, control, llegada y registro) y anade:
  - Percepcion: intensidad IR -> distancia con la tabla de la sesion 04
    y filtro exponencial.
  - Potencial repulsivo por sensor, con 4 funciones seleccionables.
  - Fuerza resultante F_res = F_atr + sum(F_rep_i) en el marco del robot.
  - Seguridad: parachoques (parada y registro) y distancia IR critica
    (se anula el avance y solo se permite girar).
"""

import argparse
import asyncio
import csv
import io
import json
import math
import os
import re
import sys
import time
from collections import deque
from dataclasses import dataclass, asdict, replace
from datetime import datetime
from pathlib import Path


PARTE = 2


# INTERFAZ DE TERMINAL
# Colores ANSI solo en una consola interactiva; NO_COLOR los desactiva.
ANCHO = 66
_TTY = sys.stdout.isatty()
_COLOR = _TTY and "NO_COLOR" not in os.environ

if _TTY and os.name == "nt":
    os.system("")  # Activa las secuencias ANSI en la consola de Windows.

if isinstance(sys.stdout, io.TextIOWrapper):
    # Una salida redirigida en Windows usa cp1252: escribir UTF-8.
    if not _TTY:
        sys.stdout.reconfigure(encoding="utf-8")
    sys.stdout.reconfigure(errors="replace")

NEGRITA, TENUE = "1", "2"
ROJO, VERDE, AMARILLO, CIAN = "31", "32", "33", "36"

_ANSI = re.compile(r"\x1b\[[0-9;]*m")
_linea_viva = False


def estilo(texto, *codigos):
    if not _COLOR or not codigos:
        return str(texto)
    return f"\x1b[{';'.join(codigos)}m{texto}\x1b[0m"


def _visible(texto):
    return len(_ANSI.sub("", texto))


def _junto(*partes):
    return estilo(" · ", TENUE).join(partes)


def _cerrar_linea():
    """Terminar la linea de estado en vivo antes de escribir otra cosa."""
    global _linea_viva
    if _linea_viva:
        print()
        _linea_viva = False


def cabecera(titulo, subtitulo=None):
    _cerrar_linea()
    borde = "─" * (ANCHO - 2)

    def en_caja(texto):
        relleno = " " * max(0, ANCHO - 4 - _visible(texto))
        return f"{estilo('│', CIAN)} {texto}{relleno} {estilo('│', CIAN)}"

    print()
    print(estilo(f"╭{borde}╮", CIAN))
    print(en_caja(estilo(titulo, NEGRITA)))
    if subtitulo:
        print(en_caja(estilo(subtitulo, TENUE)))
    print(estilo(f"╰{borde}╯", CIAN))


def seccion(titulo):
    _cerrar_linea()
    texto = f"── {titulo} "
    print()
    print(
        "  " + estilo(texto, CIAN, NEGRITA)
        + estilo("─" * max(0, ANCHO - 2 - len(texto)), CIAN)
    )


def _mensaje(simbolo, texto, color):
    _cerrar_linea()
    print(f"  {estilo(simbolo, color, NEGRITA)} {texto}")


def ok(texto):
    _mensaje("✓", texto, VERDE)


def aviso(texto):
    _mensaje("!", texto, AMARILLO)


def fallo(texto):
    _mensaje("✗", texto, ROJO)


def info(texto):
    _mensaje("•", texto, CIAN)


def nota(texto):
    _cerrar_linea()
    print(f"    {estilo(texto, TENUE)}")


def fila(etiqueta, valor):
    print(f"    {estilo(f'{etiqueta:<27}', TENUE)}{valor}")


def preguntar(texto, actual=None):
    pista = "" if actual is None else " " + estilo(f"[{actual}]", TENUE)
    return input(f"  {estilo('›', CIAN, NEGRITA)} {texto}{pista}: ")


def apartado(clave, nombre, *lineas):
    """Entrada del resumen: [clave] nombre y una o varias lineas de valores."""
    marca = estilo(f"[{clave}]", CIAN, NEGRITA) if clave else "   "
    print(f"  {marca} {estilo(f'{nombre:<20}', NEGRITA)}{lineas[0]}")
    for linea in lineas[1:]:
        print(" " * 26 + linea)


def lista_opciones(opciones, actual):
    """opciones: (tecla, valor interno, nombre, descripcion)."""
    for tecla, valor, nombre, descripcion in opciones:
        marca = estilo("  ● actual", VERDE) if valor == actual else ""
        print(
            f"  {estilo(f'[{tecla}]', CIAN, NEGRITA)} "
            f"{nombre:<20}{estilo(descripcion, TENUE)}{marca}"
        )
    print()


_ESTADOS = {
    "NAVEGACION": ("Navegando", CIAN),
    "EMERGENCIA": ("IR critico", ROJO),
    "MINIMO_LOCAL": ("Min. local", ROJO),
    "ESCAPE_TANGENCIAL": ("Contorno", AMARILLO),
    "ESCAPE_SUBMETA": ("Submeta", AMARILLO),
    "FRENANDO": ("Frenando", AMARILLO),
    "ORIENTACION_FINAL": ("Orientando", AMARILLO),
    "LLEGADA": ("En meta", VERDE),
}


def mostrar_estado(t, pose, distancia, estado, v, d_ir):
    """Linea de estado que se reescribe en cada ciclo de control."""
    global _linea_viva
    if not _TTY:
        return

    nombre, color = _ESTADOS.get(estado, (estado, CIAN))
    ir = "  -- " if not math.isfinite(d_ir) else f"{d_ir*100:5.1f}"
    texto = (
        f"  {estilo(f'{nombre:<10}', color, NEGRITA)} "
        f"t {t:5.1f} s  "
        f"({pose.x*100:6.1f},{pose.y*100:6.1f}) cm  "
        f"θ {math.degrees(pose.theta):4.0f}°  "
        f"d {distancia*100:5.1f} cm  "
        f"v {v*100:4.1f} cm/s  "
        f"IR {ir} cm"
    )
    print("\r\x1b[2K" + texto, end="", flush=True)
    _linea_viva = True


def _cm(valor, unidad="cm"):
    return "N/D" if valor is None else f"{valor*100:.1f} {unidad}"


def mostrar_resultado(datos, carpeta):
    seccion("Resultado")

    if datos["resultado"] == "meta_alcanzada":
        ok(estilo("Meta alcanzada", VERDE, NEGRITA))
    else:
        fallo(estilo(datos["resultado"], ROJO, NEGRITA))

    print()
    fila("Tiempo", f"{datos['tiempo_s']:.1f} s")
    fila("Error al decidir parar", _cm(datos["error_final_m"]))
    fila("Error tras parar (odom.)", _cm(datos.get("error_tras_parada_m")))
    fila("Colision (parachoques)", "SI" if datos["colision"] else "no")
    fila("Distancia minima IR", _cm(datos["distancia_minima_ir_m"]))
    fila("Avisos de IR critico", datos["emergencias_ir"])
    if PARTE == 3:
        fila("Escapes activados", datos["escapes"])
    fila("Vmax ordenada", _cm(datos["vmax_ordenada_m_s"], "cm/s"))
    fila(
        "Vmax estimada (odometria)",
        _cm(datos["vmax_estimada_odometria_m_s"], "cm/s")
    )
    fila("Ciclos retrasados", datos["ciclos_retrasados"])
    fila("Registro", estilo(carpeta, TENUE))


def avisos_inicio(c):
    seccion("Antes de iniciar")
    aviso("Coloque el robot en el origen, fuera de la base.")
    aviso("Coloque los obstaculos del escenario y despeje el resto.")
    aviso(
        "Angulos IR en uso (orden del SDK): "
        + " ".join(f"{math.degrees(a):g}" for a in c.angulos_ir)
        + " °"
    )
    nota("Si no se han verificado (indicaciones 2.2.3), hagalo antes.")
    info(
        "La odometria se adapta al angulo inicial indicado; "
        "no se gira para alinearla."
    )
    print()


# CONFIGURACION
# Angulos aproximados de los 7 IR frontales del Create 3 respecto a x_R
# (0 = delante, positivo = izquierda), del sensor derecho al izquierdo.
# VERIFICAR el orden real del vector del SDK (indicaciones 2.2.3).
ANGULOS_IR_DEFECTO = tuple(
    math.radians(a) for a in (-65.3, -38.0, -20.0, -3.0, 14.25, 34.0, 65.3)
)

# clave: (nombre, formula, ganancia propuesta).
# Las ganancias tienen unidades distintas en cada funcion; las propuestas
# dan aproximadamente la misma fuerza (1.2) a 10 cm con d0 = 25 cm.
REPULSIVOS = {
    "cuadratico": (
        "Cuadratico (Khatib)", "U = k/2·(1/d - 1/d0)²", 0.002
    ),
    "inverso": (
        "Inverso", "U = k·(1/d - 1/d0)", 0.012
    ),
    "lineal_tramos": (
        "Lineal por tramos", "F = k·(d0 - d) si d < d0; 0 fuera", 8.0
    ),
    "propio": (
        "Propio (suave)", "F = k·(1 - d/d0)²/d", 0.33
    ),
}


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
    vmax: float = 0.20  # Recomendado en las indicaciones: <= 0.20 m/s.
    wmax: float = 0.8
    rueda_max: float = 0.50
    aceleracion: float = 0.10  # m/s2.
    frenado: float = 0.10  # m/s2: parametro del perfil de aproximacion.

    separacion_ruedas: float = 0.235  # Medir antes de validar.
    tolerancia: float = 0.05
    tolerancia_angular: float = math.radians(3)
    # Periodo real: leer pose + IR y enviar ruedas por Bluetooth.
    periodo: float = 0.30
    # Retraso entre la pose leida y el efecto de la orden (ver P01).
    latencia: float = 0.30
    tiempo_max: float = 180.0
    timeout_lectura: float = 2.0

    # Repulsion (activa desde P02).
    repulsivo: str = "cuadratico"
    ganancia_repulsiva: float = 0.002
    influencia: float = 0.25

    # Percepcion IR.
    angulos_ir: tuple = ANGULOS_IR_DEFECTO  # Radianes, orden del SDK.
    filtro_alpha: float = 0.5
    seguridad: float = 0.06
    fuerza_max_sensor: float = 2.0
    calibracion_json: str = ""  # Vacio: tabla de la sesion 04.
    memoria_obstaculos: float = 5.0  # s que se recuerda cada deteccion.
    radio_robot: float = 0.17  # Create 3: 34 cm de diametro.

    def validar(self):
        for nombre, valor in asdict(self).items():
            if isinstance(valor, (int, float)) and not math.isfinite(valor):
                raise ValueError(f"{nombre}: debe ser finito")

        for nombre in (
            "ganancia", "transicion", "kv", "kw", "vmax", "wmax",
            "rueda_max", "separacion_ruedas", "tolerancia",
            "tolerancia_angular", "periodo", "tiempo_max",
            "timeout_lectura", "ganancia_repulsiva", "influencia",
            "aceleracion", "frenado", "latencia", "filtro_alpha",
            "seguridad", "fuerza_max_sensor", "memoria_obstaculos",
            "radio_robot"
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

        if self.repulsivo not in REPULSIVOS:
            raise ValueError("Funcion repulsiva desconocida")

        if len(self.angulos_ir) != 7 or not all(
            math.isfinite(a) for a in self.angulos_ir
        ):
            raise ValueError("Se necesitan 7 angulos IR finitos")

        if any(abs(a) > math.pi/2 + 1e-6 for a in self.angulos_ir):
            raise ValueError(
                "Los sensores frontales deben estar entre -90 y +90 grados"
            )

        if len(set(self.angulos_ir)) != 7:
            raise ValueError("Cada sensor debe tener un angulo distinto")

        if self.filtro_alpha > 1:
            raise ValueError("alpha debe estar entre 0 (excluido) y 1")

        if not 0.05 < self.seguridad < self.influencia <= 0.25:
            raise ValueError(
                "Se exige 5 cm < seguridad < influencia <= 25 cm "
                "(rango de la calibracion IR)"
            )

        return self


def pedir_numero(texto, actual, positivo=False):
    """Intro conserva el valor. Una entrada incorrecta se vuelve a pedir."""
    while True:
        entrada = preguntar(
            texto, f"{actual:g}"
        ).strip().replace(",", ".")

        try:
            valor = actual if not entrada else float(entrada)

            if not math.isfinite(valor) or (positivo and valor <= 0):
                raise ValueError

            return valor

        except ValueError:
            fallo(
                "Introduzca un numero finito"
                + (" mayor que cero." if positivo else ".")
            )


ROSA_VIENTOS = (
    "              +Y (90°)",
    "                  ▲",
    "    -X (180°) ◄───┼───► +X (0°)",
    "                  ▼",
    "              -Y (-90°)",
)


def pedir_orientacion_inicial():
    """Exigir un angulo escrito; no suponer que el robot mira hacia +Y."""
    print()
    for linea in ROSA_VIENTOS:
        print("  " + estilo(linea, CIAN))
    print()
    nota("Ejes fijos del suelo. El robot empieza en (0, 0),")
    nota("pero puede mirar en cualquier direccion.")
    print()

    while True:
        entrada = preguntar(
            "Orientacion INICIAL del robot (grados)"
        ).strip()

        try:
            valor = float(entrada.replace(",", "."))

            if not math.isfinite(valor):
                raise ValueError

            return math.radians(valor)

        except ValueError:
            fallo(
                "Es obligatorio escribir un angulo finito, "
                "por ejemplo 45 o -90."
            )


def mostrar_configuracion(c, menu=True):
    """Resumen de la mision. En el menu cada apartado lleva su tecla."""
    def tecla(k):
        return k if menu else ""

    final = (
        "libre"
        if c.meta_theta is None
        else f"{math.degrees(c.meta_theta):g}°"
    )

    cabecera(
        "Configuracion de la mision",
        "Origen (0, 0) inicial · X e Y: ejes fijos del suelo"
    )
    print()

    apartado(
        tecla("0"), "Orientacion inicial",
        f"{math.degrees(c.orientacion_inicial):g}°",
        estilo("0° = +X · 90° = +Y · 180° = -X · -90° = -Y", TENUE)
    )
    apartado(
        tecla("1"), "Meta",
        _junto(f"X = {c.meta_x*100:g} cm", f"Y = {c.meta_y*100:g} cm"),
        f"orientacion final: {final}"
    )
    apartado(
        tecla("2"), "Potencial atractivo",
        _junto(
            c.potencial,
            f"k = {c.ganancia:g}",
            f"transicion = {c.transicion*100:g} cm"
        )
    )
    apartado(
        tecla("3"), "Control",
        _junto(f"kv = {c.kv:g}", f"kw = {c.kw:g}"),
        _junto(
            f"avance ≤ {c.vmax*100:g} cm/s",
            f"rueda ≤ {c.rueda_max*100:g} cm/s"
        ),
        f"giro ≤ {math.degrees(c.wmax):g}°/s",
        _junto(
            f"aceleracion {c.aceleracion*100:g} cm/s²",
            f"frenado {c.frenado*100:g} cm/s²"
        )
    )
    apartado(
        tecla("4"), "Llegada",
        _junto(
            f"error ≤ {c.tolerancia*100:g} cm",
            f"error angular ≤ {math.degrees(c.tolerancia_angular):g}°"
        ),
        _junto(
            f"periodo {c.periodo:g} s",
            f"latencia {c.latencia:g} s",
            f"tiempo maximo {c.tiempo_max:g} s"
        )
    )
    apartado(
        tecla("5"), "Robot",
        c.robot,
        _junto(
            f"separacion ruedas {c.separacion_ruedas*100:g} cm",
            f"timeout {c.timeout_lectura:g} s"
        )
    )
    nombre, formula, _ = REPULSIVOS[c.repulsivo]
    apartado(
        tecla("6"), "Potencial repulsivo",
        _junto(
            nombre,
            f"kr = {c.ganancia_repulsiva:g}",
            f"d0 = {c.influencia*100:g} cm"
        ),
        estilo(formula, TENUE),
        f"relacion kr/ka = {c.ganancia_repulsiva/c.ganancia:g}"
    )
    apartado(
        tecla("7"), "Sensores IR",
        "angulos " + " ".join(
            f"{math.degrees(a):g}" for a in c.angulos_ir
        ) + " °",
        _junto(
            f"filtro α = {c.filtro_alpha:g}",
            f"seguridad {c.seguridad*100:g} cm",
            f"F max/sensor {c.fuerza_max_sensor:g}"
        ),
        _junto(
            f"memoria {c.memoria_obstaculos:g} s",
            f"radio robot {c.radio_robot*100:g} cm"
        ),
        "calibracion: " + (c.calibracion_json or "tabla de la sesion 04")
    )
    mostrar_configuracion_extra(c, tecla)

    if menu:
        print()
        print("  " + estilo("─" * (ANCHO - 4), TENUE))
        print(
            f"  {estilo('[I]', VERDE, NEGRITA)} Iniciar mision    "
            f"{estilo('[Q]', ROJO, NEGRITA)} Salir sin conectar    "
            f"{estilo(f'[0-{max(EDITORES)}]', CIAN, NEGRITA)} Editar"
        )


def mostrar_configuracion_extra(c, tecla):
    """P02 no tiene apartados extra; P03 anade el escape."""


def editar_orientacion(c):
    return replace(
        c,
        orientacion_inicial=pedir_orientacion_inicial()
    ).validar()


def editar_meta(c):
    x = pedir_numero("Meta X (cm)", c.meta_x*100) / 100
    y = pedir_numero("Meta Y (cm)", c.meta_y*100) / 100

    nota(
        "Rumbo absoluto del escenario: "
        "0 = +X, 90 = +Y, 180 = -X, -90 = -Y."
    )
    nota("'no' desactiva la orientacion final.")

    actual = (
        "no"
        if c.meta_theta is None
        else f"{math.degrees(c.meta_theta):g}"
    )

    while True:
        valor = preguntar(
            "Orientacion final (grados)", actual
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
            fallo("Introduzca un angulo finito o 'no'.")


def editar_potencial(c):
    opciones = (
        ("1", "cuadratico", "Cuadratico",
         "atraccion proporcional a la distancia"),
        ("2", "conico", "Conico",
         "magnitud constante fuera de la meta"),
        ("3", "hibrido", "Hibrido",
         "cuadratico cerca, conico lejos"),
        ("4", "suave", "Propio (suave)",
         "transicion gradual, atraccion limitada"),
    )
    lista_opciones(opciones, c.potencial)
    claves = {tecla: valor for tecla, valor, *_ in opciones}

    while True:
        entrada = preguntar("Potencial 1-4", c.potencial).strip()

        if not entrada or entrada in claves:
            potencial = claves.get(entrada, c.potencial)
            break

        fallo("Elija una opcion del 1 al 4.")

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
                fallo("El maximo admitido por el SDK es 50 cm/s.")
                continue

            if nombre == "vmax" and valor > 0.20:
                aviso("Las indicaciones recomiendan vmax ≤ 20 cm/s.")

            cambios[nombre] = valor
            break

    return replace(c, **cambios).validar()


def editar_control(c):
    return editar_parametros(c, (
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


def editar_llegada(c):
    return editar_parametros(c, (
        ("tolerancia", "Error de posicion permitido (cm)", 100),
        (
            "tolerancia_angular",
            "Error angular permitido (grados)",
            180/math.pi
        ),
        ("periodo", "Periodo de control (s)", 1),
        ("latencia", "Latencia de parada anticipada (s)", 1),
        ("tiempo_max", "Duracion maxima de la mision (s)", 1)
    ))


def editar_robot(c):
    robot = preguntar("Nombre Bluetooth", c.robot).strip() or c.robot

    return editar_parametros(replace(c, robot=robot), (
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


def editar_repulsion(c):
    opciones = tuple(
        (str(i), clave, nombre, formula)
        for i, (clave, (nombre, formula, _)) in enumerate(
            REPULSIVOS.items(), 1
        )
    )
    lista_opciones(opciones, c.repulsivo)
    claves = {tecla: valor for tecla, valor, *_ in opciones}

    while True:
        opcion = preguntar("Funcion 1-4", c.repulsivo).strip()

        if not opcion or opcion in claves:
            break

        fallo("Seleccione 1-4.")

    repulsivo = claves.get(opcion, c.repulsivo)
    ganancia = c.ganancia_repulsiva

    if repulsivo != c.repulsivo:
        # Cada funcion tiene unidades distintas: proponer una equivalente.
        ganancia = REPULSIVOS[repulsivo][2]
        nota(f"Ganancia propuesta para esta funcion: {ganancia:g}")

    return editar_parametros(
        replace(c, repulsivo=repulsivo, ganancia_repulsiva=ganancia),
        (
            ("ganancia_repulsiva", "Ganancia repulsiva kr", 1),
            ("influencia", "Distancia de influencia d0 (cm, ≤ 25)", 100)
        )
    )


def editar_sensores(c):
    nota("Angulos en grados, en el ORDEN del vector del SDK.")
    nota("0 = delante, positivo = izquierda. Separados por espacios.")
    actual = " ".join(f"{math.degrees(a):g}" for a in c.angulos_ir)

    while True:
        entrada = preguntar("7 angulos IR", actual).strip()

        try:
            angulos = (
                c.angulos_ir
                if not entrada
                else tuple(
                    math.radians(float(v.replace(",", ".")))
                    for v in entrada.split()
                )
            )
            replace(c, angulos_ir=angulos).validar()
            break

        except ValueError as error:
            fallo(f"Angulos no validos: {error or 'formato'}")

    nota("Fichero JSON {distancia_cm: [7 intensidades]} entre 5 y 25 cm.")
    nota("'-' vuelve a la tabla de la sesion 04.")

    while True:
        ruta = preguntar(
            "Calibracion IR",
            c.calibracion_json or "sesion 04"
        ).strip()

        if not ruta:
            ruta = c.calibracion_json
        elif ruta == "-":
            ruta = ""

        try:
            cargar_calibracion(ruta)
            break
        except (OSError, ValueError, TypeError) as error:
            fallo(f"Calibracion no valida: {error}")

    return editar_parametros(
        replace(c, angulos_ir=angulos, calibracion_json=ruta),
        (
            ("filtro_alpha", "Filtro exponencial alpha (0-1]", 1),
            (
                "seguridad",
                "Distancia IR critica: solo giro (cm, > 5)",
                100
            ),
            ("fuerza_max_sensor", "Fuerza repulsiva maxima por sensor", 1),
            (
                "memoria_obstaculos",
                "Memoria de obstaculos vistos (s)",
                1
            ),
            ("radio_robot", "Radio del robot (cm)", 100)
        )
    )


EDITORES = {
    "0": ("Orientacion inicial", editar_orientacion),
    "1": ("Meta", editar_meta),
    "2": ("Potencial atractivo", editar_potencial),
    "3": ("Control", editar_control),
    "4": ("Llegada", editar_llegada),
    "5": ("Robot", editar_robot),
    "6": ("Potencial repulsivo", editar_repulsion),
    "7": ("Sensores IR", editar_sensores),
}


def menu_configuracion(c):
    redibujar = True

    while True:
        if redibujar:
            mostrar_configuracion(c)
            print()
        redibujar = True

        opcion = preguntar("Opcion").strip().lower()

        if opcion in EDITORES:
            titulo, editor = EDITORES[opcion]
            seccion(titulo)
            if opcion != "0":
                nota("Intro conserva el valor actual.")

            try:
                c = editor(c)
            except ValueError as error:
                fallo(f"Configuracion no valida: {error}")

        elif opcion == "i":
            avisos_inicio(c)

            confirmacion = preguntar(
                "Escriba SI para conectar e iniciar, "
                "o Intro para volver"
            ).strip().lower()

            if confirmacion == "si":
                return c.validar()

        elif opcion == "q":
            print()
            aviso("Mision cancelada sin conectar al robot.")
            raise SystemExit(1)

        else:
            fallo(f"Opcion no valida: elija 0-{max(EDITORES)}, I o Q.")
            redibujar = False


def anadir_argumentos_extra(p):
    """P02 no tiene argumentos extra; P03 anade los del escape."""


def configuracion_extra(a):
    """Campos extra de la configuracion a partir de los argumentos."""
    return {}


def configurar():
    p = argparse.ArgumentParser(
        description=f"Parte {PARTE:02}: campos de potencial con obstaculos."
    )

    p.add_argument("--robot", default="C3_UIEC_Grupo4")
    p.add_argument("--x-cm", type=float, help="Meta X en el escenario")
    p.add_argument("--y-cm", type=float, help="Meta Y en el escenario")
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
    p.add_argument("--vmax-cm-s", type=float, default=20.0)
    p.add_argument(
        "--wmax-grados-s",
        type=float,
        default=math.degrees(0.8)
    )
    p.add_argument("--rueda-max-cm-s", type=float, default=50.0)
    p.add_argument("--aceleracion-cm-s2", type=float, default=10.0)
    p.add_argument("--frenado-cm-s2", type=float, default=10.0)
    p.add_argument("--separacion-cm", type=float, default=23.5)
    p.add_argument("--tolerancia-cm", type=float, default=5.0)
    p.add_argument("--periodo", type=float, default=0.30)
    p.add_argument("--latencia", type=float, default=0.30)
    p.add_argument("--tiempo-max", type=float, default=180.0)
    p.add_argument("--tolerancia-angular-grados", type=float, default=3.0)
    p.add_argument("--timeout-lectura", type=float, default=2.0)
    p.add_argument(
        "--repulsivo",
        choices=list(REPULSIVOS),
        default="cuadratico"
    )
    p.add_argument(
        "--ganancia-repulsiva",
        type=float,
        help="Por defecto, la propuesta para la funcion elegida"
    )
    p.add_argument("--influencia-cm", type=float, default=25.0)
    p.add_argument(
        "--angulos-grados",
        type=float,
        nargs=7,
        help="7 angulos IR en orden del SDK: 0 delante, + izquierda"
    )
    p.add_argument("--alpha", type=float, default=0.5)
    p.add_argument("--seguridad-cm", type=float, default=6.0)
    p.add_argument("--fuerza-max-sensor", type=float, default=2.0)
    p.add_argument("--memoria-obstaculos", type=float, default=5.0)
    p.add_argument(
        "--calibracion-json",
        default="",
        help="Tabla {distancia_cm: [s1..s7]} entre 5 y 25 cm"
    )
    anadir_argumentos_extra(p)
    p.add_argument(
        "--sin-menu",
        action="store_true",
        help="Usar argumentos y confirmacion breve"
    )

    a = p.parse_args()

    cabecera(
        f"PARTE {PARTE:02} · "
        + ("MINIMOS LOCALES" if PARTE == 3 else "REPULSION"),
        "Campos de potencial con obstaculos · Grupo 4"
    )

    try:
        seccion("Orientacion inicial del robot")
        orientacion_inicial = pedir_orientacion_inicial()

        if a.x_cm is None or a.y_cm is None:
            seccion("Meta")
            nota("Intro conserva el valor propuesto.")

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

        angulos = (
            ANGULOS_IR_DEFECTO
            if a.angulos_grados is None
            else tuple(math.radians(v) for v in a.angulos_grados)
        )

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
            latencia=a.latencia,
            tiempo_max=a.tiempo_max,
            tolerancia_angular=math.radians(a.tolerancia_angular_grados),
            timeout_lectura=a.timeout_lectura,
            repulsivo=a.repulsivo,
            ganancia_repulsiva=(
                REPULSIVOS[a.repulsivo][2]
                if a.ganancia_repulsiva is None
                else a.ganancia_repulsiva
            ),
            influencia=a.influencia_cm/100,
            angulos_ir=angulos,
            filtro_alpha=a.alpha,
            seguridad=a.seguridad_cm/100,
            fuerza_max_sensor=a.fuerza_max_sensor,
            memoria_obstaculos=a.memoria_obstaculos,
            calibracion_json=a.calibracion_json,
            **configuracion_extra(a)
        ).validar()

        cargar_calibracion(c.calibracion_json)

    except (ValueError, TypeError, OSError, EOFError) as error:
        p.error(str(error))

    if not a.sin_menu:
        return menu_configuracion(c)

    mostrar_configuracion(c, menu=False)
    avisos_inicio(c)
    preguntar("Pulse Intro para conectar e iniciar; Ctrl+C cancela")

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


class MemoriaControl:
    """Estado del controlador entre ciclos; independiente del SDK.

    actual: velocidad de avance efectiva del ciclo anterior (rampa y
            parada anticipada).
    giro:   ultimo sentido de giro (histeresis con la fuerza detras).
    """

    def __init__(self):
        self.actual = 0.0
        self.giro = 0.0

    def limitar(self, objetivo, dt, config):
        """Limita los aumentos; las reducciones se aplican al momento."""
        # Un retraso Bluetooth no debe provocar un salto grande al reanudar.
        dt = max(0.0, min(dt, config.periodo))

        return min(
            objetivo,
            self.actual + config.aceleracion*dt
        )


# Con la fuerza casi detras, atan2 salta entre +180 y -180 grados y el
# giro cambiaria de sentido en cada ciclo (el robot se quedaria
# oscilando sin avanzar). Por encima de este angulo se conserva el
# sentido de giro anterior.
HISTERESIS = math.radians(150)


def velocidades(fx, fy, distancia, config, memoria=None):
    """v y w se calculan a la vez en cada ciclo a partir de F_res.

    v se multiplica por max(0, cos(error)): si la fuerza apunta hacia
    atras (obstaculo delante) no se avanza hacia el obstaculo, pero el
    giro sigue activo en el mismo ciclo.
    """
    error = math.atan2(fy, fx)

    if memoria is not None and memoria.giro and abs(error) > HISTERESIS:
        error = math.copysign(abs(error), memoria.giro)

    w = max(
        -config.wmax,
        min(config.wmax, config.kw*error)
    )

    # Reservar distancia para lo que se avanza sin poder corregir
    # (un ciclo + la latencia) y para el frenado:
    # v*(Ts + L) + v^2/(2*b) <= distancia a la meta.
    disponible = max(0.0, distancia)
    bt = config.frenado * (config.periodo + config.latencia)

    limite_frenado = (
        math.sqrt(bt*bt + 2*config.frenado*disponible) - bt
    )

    v = min(
        config.vmax,
        config.kv*math.hypot(fx, fy),
        config.kv*distancia,
        limite_frenado
    )

    return v*max(0.0, math.cos(error)), w



def velocidad_segura(obstaculos, config):
    """Avance maximo para poder parar antes de la distancia de seguridad.

    Entre que se lee un IR y la orden de parar hace efecto pasan unos
    (periodo + latencia) segundos. En ese tiempo el robot se acerca a
    cada obstaculo v*cos(a) por segundo, y esa distancia no debe superar
    la holgura (d - seguridad). Asi el robot frena solo al acercarse,
    en vez de llegar rapido y depender de la parada de emergencia.
    """
    reaccion = config.periodo + config.latencia
    limite = config.vmax

    for d, a in obstaculos:
        acercamiento = math.cos(a)
        if acercamiento > 0.1:
            holgura = max(0.0, d - config.seguridad)
            limite = min(limite, holgura/(reaccion*acercamiento))

    return limite

def ruedas(v, w, config):
    """Velocidades tangenciales de ruedas en m/s.

    Si una rueda supera el limite se escalan ambas por el mismo factor
    para conservar la curvatura. Devuelve tambien v y w efectivas.
    """
    izq = v - w*config.separacion_ruedas/2
    der = v + w*config.separacion_ruedas/2

    escala = max(
        1.0,
        abs(izq)/config.rueda_max,
        abs(der)/config.rueda_max
    )

    return izq/escala, der/escala, v/escala, w/escala


# POTENCIAL ATRACTIVO
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


# PERCEPCION: MUESTRAS DEL EQUIPO (T03 / SESION 04), NO UNIVERSALES.
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
    """Intensidad -> distancia por interpolacion lineal entre muestras."""

    def __init__(self, muestras):
        self.muestras = dict(sorted(muestras.items()))

        if (
            not self.muestras
            or min(self.muestras) != 5
            or max(self.muestras) != 25
        ):
            raise ValueError("Se requieren muestras desde 5 hasta 25 cm")

        if any(not math.isfinite(d) or d <= 0 for d in self.muestras):
            raise ValueError("Distancias de calibracion invalidas")

        for fila_ir in self.muestras.values():
            if len(fila_ir) != 7 or any(
                not math.isfinite(v) or v <= 0 for v in fila_ir
            ):
                raise ValueError(
                    "Cada distancia requiere 7 intensidades positivas"
                )

        self.tablas = [
            tuple((fila_ir[i], d/100) for d, fila_ir in self.muestras.items())
            for i in range(7)
        ]

        if any(
            any(a[0] <= b[0] for a, b in zip(t, t[1:]))
            for t in self.tablas
        ):
            raise ValueError(
                "La intensidad de cada sensor debe disminuir con la distancia"
            )

    def convertir(self, valores):
        if len(valores) != 7 or any(
            v is None or not math.isfinite(v) or v < 0 for v in valores
        ):
            raise RuntimeError("Lectura IR invalida; se detiene la mision")

        return tuple(
            self.distancia(v, t) for v, t in zip(valores, self.tablas)
        )

    @staticmethod
    def distancia(valor, tabla):
        if valor < tabla[-1][0]:
            return math.inf  # Sin deteccion util.
        if valor >= tabla[0][0]:
            return tabla[0][1]  # Saturacion cercana.
        for (a, da), (b, db) in zip(tabla, tabla[1:]):
            if b <= valor <= a:
                return da + (a - valor)/(a - b)*(db - da)
        raise RuntimeError("No se pudo interpolar la lectura IR")


def cargar_calibracion(ruta):
    """Ruta vacia: tabla de la sesion 04. Si no, JSON {cm: [7 valores]}."""
    if not ruta:
        return CalibradorIR(MUESTRAS_IR)

    datos = json.loads(Path(ruta).read_text(encoding="utf-8"))
    return CalibradorIR({float(k): v for k, v in datos.items()})


class FiltroIR:
    """Filtro exponencial d_f = alpha*d + (1-alpha)*d_f_anterior.

    Una deteccion nueva entra de inmediato y una desaparicion sale de
    inmediato: nunca se mezcla infinito con una distancia.
    """

    def __init__(self, alpha):
        self.alpha = alpha
        self.anteriores = None

    def actualizar(self, distancias):
        if self.anteriores is None:
            self.anteriores = distancias

        self.anteriores = tuple(
            d if not math.isfinite(d) or not math.isfinite(prev)
            else self.alpha*d + (1 - self.alpha)*prev
            for d, prev in zip(distancias, self.anteriores)
        )
        return self.anteriores


SECTORES = 12  # Sectores de 30 grados alrededor del robot.


class MapaLocal:
    """Memoria corta de los obstaculos vistos por los IR.

    Los 7 IR solo miran hacia delante (unos ±65 grados). Al girar para
    esquivar, el obstaculo sale de su campo de vision, la repulsion
    desaparece y la atraccion devuelve al robot contra el: oscilacion
    y roce con el costado. Por eso cada deteccion se guarda como un
    punto en los ejes del escenario durante 'memoria_obstaculos'
    segundos, y la repulsion se calcula desde esos puntos.

    Para no sumar muchas veces el mismo obstaculo, se usa solo el
    punto mas cercano de cada sector de 30 grados alrededor del robot.
    """

    def __init__(self):
        self.puntos = []  # (x, y, t) en ejes del escenario.

    def actualizar(self, pose, distancias, t, config):
        """Anade las detecciones del ciclo y olvida las caducadas.

        Los obstaculos son fijos: un punto solo se olvida por tiempo o
        por quedar lejos. No se borra cuando un IR deja de verlo, porque
        el canto de un panel puede quedar entre dos sensores.
        """
        radio = config.radio_robot
        d0 = config.influencia

        conservados = [
            (x, y, t0) for x, y, t0 in self.puntos
            if t - t0 <= config.memoria_obstaculos
            and self._local(pose, x, y, radio)[0] <= 2*d0
        ]

        for d, a in zip(distancias, config.angulos_ir):
            if math.isfinite(d) and d < d0:
                rumbo = pose.theta + a
                conservados.append((
                    pose.x + (radio + d)*math.cos(rumbo),
                    pose.y + (radio + d)*math.sin(rumbo),
                    t
                ))

        self.puntos = conservados[-400:]

    @staticmethod
    def _local(pose, x, y, radio):
        dx, dy = x - pose.x, y - pose.y
        borde = math.hypot(dx, dy) - radio
        angulo = angulo_corto(math.atan2(dy, dx) - pose.theta)
        return borde, angulo

    def obstaculos(self, pose, config):
        """[(distancia al borde, angulo local)], uno por sector.

        Se ignoran los puntos que quedan detras (mas de 110 grados):
        el robot no avanza hacia atras y empujarian hacia delante.
        """
        ancho = 2*math.pi/SECTORES
        cercanos = {}

        for x, y, _ in self.puntos:
            borde, angulo = self._local(pose, x, y, config.radio_robot)
            if borde >= config.influencia or abs(angulo) > math.radians(110):
                continue

            sector = int((angulo + math.pi)//ancho) % SECTORES
            if sector not in cercanos or borde < cercanos[sector][0]:
                cercanos[sector] = (max(borde, 0.0), angulo)

        return list(cercanos.values())


# POTENCIAL REPULSIVO: VECTORES EN EL MARCO LOCAL DEL ROBOT.
def repulsion(obstaculos, config):
    """Devuelve (U_rep, Frep_x, Frep_y) sumando los obstaculos.

    obstaculos: [(d_i, a_i)] distancia al borde y angulo local.
    F_rep,i = -f_i (cos a_i, sin a_i): aleja del obstaculo detectado.
    Todas las funciones son cero fuera de la distancia de influencia d0.
    La fuerza de cada obstaculo se limita (regularizacion del control).
    """
    fx = fy = u = 0.0
    k = config.ganancia_repulsiva
    d0 = config.influencia

    for distancia, angulo in obstaculos:
        if not math.isfinite(distancia) or distancia >= d0:
            continue

        d = max(distancia, 0.005)

        if config.repulsivo == "cuadratico":
            # Khatib: cuadratico con distancia de influencia.
            u_i = 0.5*k*(1/d - 1/d0)**2
            f = k*(1/d - 1/d0)/(d*d)

        elif config.repulsivo == "inverso":
            # Inversamente proporcional a la distancia (U continua en d0).
            u_i = k*(1/d - 1/d0)
            f = k/(d*d)

        elif config.repulsivo == "lineal_tramos":
            # Fuerza lineal dentro de d0 y nula fuera.
            u_i = 0.5*k*(d0 - d)**2
            f = k*(d0 - d)

        elif config.repulsivo == "propio":
            # Fuerza y potencial se anulan suavemente en d0 (C1) y la
            # fuerza crece como 1/d cerca: menos brusca que Khatib.
            u_i = k*(
                math.log(d0/d) - 2*(d0 - d)/d0
                + (d0*d0 - d*d)/(2*d0*d0)
            )
            f = k*(1 - d/d0)**2/d

        else:
            raise ValueError("Funcion repulsiva no implementada")

        f = min(f, config.fuerza_max_sensor)
        u += u_i
        fx -= f*math.cos(angulo)
        fy -= f*math.sin(angulo)

    return u, fx, fy


# DECISION DE CADA CICLO
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
    """P02 no gestiona minimos locales: deja la fuerza sin cambios."""
    escapes = 0

    def corregir(self, pose, t, distancia, fuerza, repulsiva,
                 obstaculos, config):
        return fuerza, "NAVEGACION", ""


def crear_supervisor(config):
    return SupervisorPasivo()


def calcular_ciclo(lectura, config, memoria, supervisor, mapa,
                   t=0.0, dt=0.0):
    """Percepcion -> fuerzas -> (v, w) -> ruedas, sin acceder al SDK."""
    pose = lectura.pose
    dx = config.meta_x - pose.x
    dy = config.meta_y - pose.y
    distancia = math.hypot(dx, dy)

    u_atr, fx, fy = atractivo(dx, dy, config)
    f_atr = a_local(fx, fy, pose.theta)

    # La repulsion usa las distancias filtradas y la memoria corta.
    mapa.actualizar(pose, lectura.filtradas, t, config)
    obstaculos = mapa.obstaculos(pose, config)
    u_rep, rx, ry = repulsion(obstaculos, config)
    f_rep = (rx, ry)
    f_res = (f_atr[0] + rx, f_atr[1] + ry)
    corregida, evento = f_res, ""

    # Parada anticipada (ver P01): restar lo que el robot avanza aun.
    distancia_prevista = distancia - memoria.actual*config.latencia

    if memoria.actual > 0 and distancia_prevista <= config.tolerancia:
        v = w = 0
        estado = "FRENANDO"

    elif distancia <= config.tolerancia:
        error = (
            0
            if config.meta_theta is None
            else angulo_corto(config.meta_theta - pose.theta)
        )

        if abs(error) <= config.tolerancia_angular:
            memoria.actual = 0.0
            return Decision(
                "LLEGADA", distancia, u_atr, u_rep, f_atr, f_rep,
                (0, 0), f_res, (0, 0, 0, 0)
            )

        # Giro sobre el eje solo para la orientacion final.
        v = 0
        w = max(-config.wmax, min(config.wmax, config.kw*error))
        estado = "ORIENTACION_FINAL"

    else:
        corregida, estado, evento = supervisor.corregir(
            pose, t, distancia, f_res, f_rep, obstaculos, config
        )
        v, w = velocidades(*corregida, distancia, config, memoria)
        v = min(memoria.limitar(v, dt, config),
                velocidad_segura(obstaculos, config))

        # Seguridad con medidas SIN filtrar (el filtro no puede retrasar
        # la reaccion) y con obstaculos recordados delante o al costado,
        # que el avance barreria. Se anula el avance; el giro sigue.
        if min(lectura.distancias) <= config.seguridad or any(
            d <= config.seguridad and abs(a) < math.radians(100)
            for d, a in obstaculos
        ):
            v = 0
            estado = "EMERGENCIA"
            evento = evento or "Distancia IR critica: se anula el avance"

    comando = ruedas(v, w, config)

    # Guardar la velocidad efectiva tras escalar las ruedas.
    memoria.actual = comando[2]
    if comando[3]:
        memoria.giro = comando[3]

    extra = (corregida[0] - f_res[0], corregida[1] - f_res[1])

    return Decision(
        estado, distancia, u_atr, u_rep, f_atr, f_rep,
        extra, corregida, comando, evento
    )


# REGISTRO
class Registro:
    CAMPOS = (
        "t_s", "Ts_s", "dt_s", "x_m", "y_m", "theta_rad",
        "distancia_meta_m", "U_atr", "U_rep",
        "F_atr_x_local", "F_atr_y_local",
        "F_rep_x_local", "F_rep_y_local",
        "F_extra_x_local", "F_extra_y_local",
        "F_res_x_local", "F_res_y_local",
        "v_m_s", "w_rad_s", "v_izq_m_s", "v_der_m_s",
        "estado", "evento", "retraso_s", "v_odom_m_s", "w_odom_rad_s"
    ) + tuple(
        f"{prefijo}{i}"
        for prefijo in ("ir_", "d_m_", "d_filtrada_m_")
        for i in range(1, 8)
    )

    def __init__(self, config, calibrador):
        self.config = config
        self.carpeta = (
            Path(__file__).resolve().parent
            / "datos"
            / Path(__file__).stem
            / datetime.now().strftime("%Y%m%d_%H%M%S_%f")
        )
        self.carpeta.mkdir(parents=True)

        for nombre, datos in (
            ("configuracion", asdict(config)),
            ("calibracion", calibrador.muestras),
        ):
            (self.carpeta / f"{nombre}.json").write_text(
                json.dumps(datos, indent=2, ensure_ascii=False),
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
        self.minima = math.inf
        self.ciclos_tarde = 0
        self.emergencias = 0
        self.estado_anterior = None
        self.muestra_anterior = None
        self.vmax_odom = None

    def ciclo(self, t, dt, lectura, decision, retraso, config):
        d, pose = decision, lectura.pose
        izq, der, v, w = d.comando

        self.vmax = max(self.vmax, abs(v))
        self.minima = min(self.minima, *lectura.distancias)
        self.ciclos_tarde += int(dt > config.periodo*1.05)

        if d.estado == "EMERGENCIA" and self.estado_anterior != "EMERGENCIA":
            self.emergencias += 1
        self.estado_anterior = d.estado

        # Estimacion por diferencias de odometria (no medida externa).
        v_odom = w_odom = None

        if self.muestra_anterior is not None:
            t0, p0 = self.muestra_anterior
            intervalo = t - t0

            if intervalo > 0:
                v_odom = math.hypot(pose.x - p0.x, pose.y - p0.y)/intervalo
                w_odom = angulo_corto(pose.theta - p0.theta)/intervalo
                self.vmax_odom = max(self.vmax_odom or 0.0, v_odom)

        self.muestra_anterior = (t, pose)

        valores = (
            t, config.periodo, dt, pose.x, pose.y, pose.theta,
            d.distancia, d.u_atr, d.u_rep,
            *d.f_atr, *d.f_rep, *d.f_extra, *d.f_res,
            v, w, izq, der,
            d.estado, d.evento, retraso, v_odom, w_odom,
            *lectura.intensidades,
            *(x if math.isfinite(x) else "" for x in lectura.distancias),
            *(x if math.isfinite(x) else "" for x in lectura.filtradas),
        )
        self.csv.writerow(dict(zip(self.CAMPOS, valores)))
        self.archivo.flush()

    def evento(self, t, motivo):
        self.csv.writerow({
            "t_s": t,
            "estado": "FIN",
            "evento": motivo
        })
        self.archivo.flush()

    def cerrar(self, motivo, tiempo, error, pose_parada, escapes, colision):
        self.archivo.close()

        error_parada = None
        if pose_parada is not None:
            error_parada = math.hypot(
                self.config.meta_x - pose_parada.x,
                self.config.meta_y - pose_parada.y
            )

        datos = {
            "resultado": motivo,
            "exito": motivo == "meta_alcanzada",
            "tiempo_s": tiempo,
            "error_final_m": error,
            # Medido 0.6 s despues de parar: incluye el deslizamiento.
            "error_tras_parada_m": error_parada,
            "pose_tras_parada": (
                None if pose_parada is None else asdict(pose_parada)
            ),
            "colision": colision,
            "distancia_minima_ir_m": (
                self.minima if math.isfinite(self.minima) else None
            ),
            "emergencias_ir": self.emergencias,
            "escapes": escapes,
            "vmax_ordenada_m_s": self.vmax,
            "vmax_estimada_odometria_m_s": self.vmax_odom,
            "ciclos_retrasados": self.ciclos_tarde,
            "nota": (
                "Distancia minima estimada por IR (frontales), no "
                "separacion geometrica de todo el chasis. Velocidad "
                "estimada por odometria. Ciclo lento: dt > 1.05*Ts."
            )
        }

        (self.carpeta / "resultado.json").write_text(
            json.dumps(datos, indent=2, ensure_ascii=False),
            encoding="utf-8"
        )

        return datos


# MISION
async def navegar(adaptador, config):
    registro = Registro(config, adaptador.calibrador)
    supervisor = crear_supervisor(config)
    memoria = MemoriaControl()
    mapa = MapaLocal()

    inicio = anterior = siguiente = time.monotonic()
    motivo, error_final = "interrumpida", None
    ultimo_evento = ""

    info(f"Registro: {estilo(registro.carpeta, TENUE)}")
    info("Navegando hacia la meta")
    print()

    try:
        while True:
            adaptador.comprobar_seguridad()
            ahora = time.monotonic()

            if ahora - inicio >= config.tiempo_max:
                raise TimeoutError("Tiempo maximo de mision alcanzado")

            lectura = await adaptador.leer_entorno()
            ahora = time.monotonic()

            decision = calcular_ciclo(
                lectura, config, memoria, supervisor, mapa,
                ahora - inicio, ahora - anterior
            )
            error_final = decision.distancia

            await adaptador.aplicar(*decision.comando[:2])

            registro.ciclo(
                ahora - inicio,
                ahora - anterior,
                lectura,
                decision,
                max(0, ahora - siguiente),
                config
            )

            if decision.evento and decision.evento != ultimo_evento:
                aviso(decision.evento)
            ultimo_evento = decision.evento

            mostrar_estado(
                ahora - inicio, lectura.pose, decision.distancia,
                decision.estado, decision.comando[2],
                min(lectura.distancias)
            )

            anterior = ahora

            if decision.estado == "LLEGADA":
                motivo = "meta_alcanzada"
                break

            siguiente += config.periodo

            # Si vamos tarde, omitir plazos vencidos sin rafagas.
            actual = time.monotonic()
            if siguiente < actual:
                siguiente += (
                    math.floor((actual - siguiente)/config.periodo) + 1
                )*config.periodo

            await asyncio.sleep(max(0, siguiente - time.monotonic()))

    except BaseException as exc:
        motivo = f"{type(exc).__name__}: {exc}"
        raise

    finally:
        # Parar antes de escribir el resumen.
        pose_parada = None
        try:
            await adaptador.parar()

            if motivo == "meta_alcanzada":
                # Releer cuando el robot ya esta quieto.
                await asyncio.sleep(0.6)
                try:
                    pose_parada = await adaptador.leer_pose()
                except Exception as exc:
                    aviso(f"No se pudo leer la pose tras parar: {exc}")
        finally:
            try:
                registro.evento(time.monotonic() - inicio, motivo)
            finally:
                datos = registro.cerrar(
                    motivo,
                    time.monotonic() - inicio,
                    error_final,
                    pose_parada,
                    supervisor.escapes,
                    adaptador.colision
                )
                mostrar_resultado(datos, registro.carpeta)

    return motivo


# ADAPTADOR DEL ROBOT: SOLO ESTA CLASE Y ejecutar() CONOCEN EL SDK.
class Adaptador:
    def __init__(self, robot, config, calibrador):
        self.robot = robot
        self.config = config
        self.calibrador = calibrador
        self.filtro = FiltroIR(config.filtro_alpha)
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

        # Copiar valores; centimetros y grados a metros y radianes.
        pose_sdk = Pose(p.x/100, p.y/100, math.radians(p.heading))

        if self.origen_sdk is None:
            # Primera lectura con el robot parado tras reset_navigation.
            self.origen_sdk = pose_sdk

        return pose_en_escenario(
            pose_sdk,
            self.origen_sdk,
            self.config.orientacion_inicial
        )

    async def leer_entorno(self):
        # Consultas secuenciales: no se presupone que el SDK serialice
        # peticiones concurrentes.
        pose = await self.leer_pose()

        ir = await asyncio.wait_for(
            self.robot.get_ir_proximity(),
            self.config.timeout_lectura
        )
        if ir is None:
            raise RuntimeError("Sensores IR no disponibles")

        valores = tuple(ir.sensors)
        distancias = self.calibrador.convertir(valores)
        self.comprobar_seguridad()

        return Lectura(
            pose, valores, distancias, self.filtro.actualizar(distancias)
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
    calibrador = cargar_calibracion(config.calibracion_json)

    robot = Create3(Bluetooth(config.robot))
    adaptador = Adaptador(robot, config, calibrador)

    @event(robot.when_bumped, [])
    async def colision(r):
        adaptador.colision = True
        await adaptador.parar()
        aviso("Parachoques activado: robot detenido.")
        await r.set_lights_blink_rgb(255, 0, 0)

    @event(robot.when_play)
    async def iniciar(r):
        try:
            ok("Conectado")
            await adaptador.parar()

            await asyncio.wait_for(
                r.reset_navigation(),
                config.timeout_lectura
            )
            await asyncio.sleep(0.2)

            await r.set_lights_on_rgb(0, 255, 0)

            # Esta primera lectura fija el origen e inicia el filtro.
            await adaptador.leer_entorno()
            ok("Origen fijado en (0, 0)")

            await r.set_lights_on_rgb(0, 0, 255)
            resultado = await navegar(adaptador, config)

            await r.set_lights_on_rgb(
                *((255, 80, 0) if resultado == "meta_alcanzada"
                  else (255, 0, 0))
            )

        except Exception as exc:
            fallo(f"Mision detenida: {exc}")

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
                    info("Robot desconectado")
                    print()

    seccion("Mision")
    info(f"Conectando por Bluetooth con {estilo(config.robot, NEGRITA)}...")

    # El SDK gestiona su bucle: no envolver play() en asyncio.run().
    robot.play()


def main():
    try:
        config = configurar()
    except (KeyboardInterrupt, EOFError):
        print()
        aviso("Configuracion cancelada; no se conecto al robot.")
        raise SystemExit(1)

    ejecutar(config)


if __name__ == "__main__":
    main()
