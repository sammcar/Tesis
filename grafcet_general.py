import threading
import time

# ========================= CONFIGURACIÓN EXTERNA ========================= #

NUM_ETAPAS = 10
SENSORES = ["p1e", "p1r", "p2e", "p2r", "p3e", "p3r"]
SALIDAS = ["P1EXT", "P1RET", "P2EXT", "P2RET", "P3EXT", "P3RET", "5 SEG", "3SEG"]

# Asociación sensor → salida
SENSOR_SALIDA_MAP = {
    "p1e": "P1EXT", "p1r": "P1RET",
    "p2e": "P2EXT", "p2r": "P2RET",
    "p3e": "P3EXT", "p3r": "P3RET",
}

# Asociación etapa → salidas
ETAPA_SALIDAS_MAP = {
    0: [],
    1: ["P1EXT"],
    2: ["P1RET"],
    3: ["P2EXT"],
    4: ["P2RET"],
    5: ["P1EXT", "P2EXT"],
    6: ["timer_5"],
    7: ["P1RET", "P2RET"],
    8: ["P3EXT", "timer_3"],
    9: ["P3RET"],
}

# Funciones asociadas a salidas (opcional)
def prender_led(color):
    print(f"[FUNC] LED {color.upper()} encendido")

SALIDA_FUNCIONES = {
    "P1EXT": lambda: prender_led("verde"),
}

# Duraciones de temporizadores por salida tipo "timer_*"
SALIDAS_TIMER_DURACION = {
    "timer_5": 5,
    "timer_3": 3,
}

# Tiempos de falla por sensor (segundos)
TIEMPO_FALLA_SENSORES = {
    "p1e": 10, "p1r": 10,
    "p2e": 10, "p2r": 10,
    "p3e": 6,  "p3r": 6,
}

# Etapas que esperan sensores
ETAPA_SENSORES_FALLO = {
    1: ["p1e"], 2: ["p1r"],
    3: ["p2e"], 4: ["p2r"],
    5: ["p1e", "p2e"],
    7: ["p1r", "p2r"],
    8: ["p3e"],
    9: ["p3r"],
}

# ========================= CLASE GRAFCET ========================= #

class GrafcetSimulador:
    def __init__(self):
        # Estado de etapas
        self.etapas = [False] * NUM_ETAPAS
        self.etapas[0] = True
        self.etapa_anterior = -1

        # Modo
        self.modo = "automatico"

        # Pulsadores y estado
        self.start = self.CLC = self.SE = self.pausa = self.fallo = False
        self.running = True

        # Salidas y sensores
        self.salidas = {s: False for s in SALIDAS}
        self.sensores = {s: False for s in SENSORES}
        self.latch = {s: False for s in SENSORES}

        # Temporizadores dinámicos por salida tipo timer
        self.timers_salida = {
            k: {"inicio": None, "terminado": False, "duracion": v}
            for k, v in SALIDAS_TIMER_DURACION.items()
        }

        # Modo manual
        self.salida_manual_actual = None
        self.inicio_manual = None

        # Timer de falla por etapa
        self.tiempo_salida = None
        self.etapa_timer_fallo = -1

    # ========================= FUNCIONES INTERNAS ========================= #

    def actualizar_timer(self):
        actual = next((i for i, e in enumerate(self.etapas) if e), -1)
        salidas_etapa = ETAPA_SALIDAS_MAP.get(actual, [])
        for salida in salidas_etapa:
            if salida.startswith("timer_"):
                timer = self.timers_salida[salida]
                if timer["inicio"] is None:
                    timer["inicio"] = time.time()
                elif (time.time() - timer["inicio"]) >= timer["duracion"]:
                    timer["terminado"] = True
        # Reset si no está en etapa con esa salida
        for salida in self.timers_salida:
            if salida not in salidas_etapa:
                self.timers_salida[salida]["inicio"] = None
                self.timers_salida[salida]["terminado"] = False

    def verificar_fallos(self):
        actual = next((i for i, e in enumerate(self.etapas) if e), -1)
        if actual != self.etapa_timer_fallo:
            self.tiempo_salida = time.time()
            self.etapa_timer_fallo = actual
        sensores_esperados = ETAPA_SENSORES_FALLO.get(actual, [])
        for sensor in sensores_esperados:
            if not self.latch[sensor] and (time.time() - self.tiempo_salida) > TIEMPO_FALLA_SENSORES.get(sensor, 10):
                print(f"❌ Falla detectada en sensor: {sensor}")
                self.fallo = True
                break

    def evo(self):
        E = self.etapas
        s = self.latch
        m = self.modo
        automatico = m == "automatico"
        ciclo = m == "ciclo"
        etapa = m == "etapa"

        transiciones = [
            (0, (automatico and self.start) or (ciclo and self.CLC) or (etapa and self.SE), 1),
            (1, (automatico or ciclo or (etapa and self.SE)) and s["p1e"], 2),
            (2, (automatico or ciclo or (etapa and self.SE)) and s["p1r"], 3),
            (3, (automatico or ciclo or (etapa and self.SE)) and s["p2e"], 4),
            (4, ((automatico or (etapa and self.SE)) and s["p2r"]) or (ciclo and s["p2r"] and self.CLC), 5),
            (5, (automatico or ciclo or (etapa and self.SE)) and s["p1e"] and s["p2e"], 6),
            (6, (automatico or ciclo or (etapa and self.SE)) and self.timers_salida["timer_5"]["terminado"], 7),
            (7, ((automatico or (etapa and self.SE)) and s["p1r"] and s["p2r"]) or (ciclo and self.CLC and s["p1r"] and s["p2r"]), 8),
            (8, (automatico or ciclo or (etapa and self.SE)) and self.timers_salida["timer_3"]["terminado"] and s["p3e"], 9),
            (9, ((automatico or ciclo or (etapa and self.SE)) and s["p3r"]), 0)
        ]

        for actual, condicion, nueva in transiciones:
            if E[actual] and condicion:
                self.etapas = [False] * NUM_ETAPAS
                self.etapas[nueva] = True
                return

    def salida(self):
        actual = next((i for i, e in enumerate(self.etapas) if e), -1)
        if actual != self.etapa_anterior:
            print(f"\n🔁 Etapa activa: {actual}")
            self.etapa_anterior = actual
        for k in self.salidas:
            self.salidas[k] = False
        for s in ETAPA_SALIDAS_MAP.get(actual, []):
            self.salidas[s] = True
            print(f"⚙️ Activando salida: {s}")
            if s in SALIDA_FUNCIONES:
                try:
                    SALIDA_FUNCIONES[s]()
                except Exception as e:
                    print(f"⚠️ Error al ejecutar función de salida {s}: {e}")

    def modo_manual(self):
        for k in self.salidas:
            self.salidas[k] = False
        if self.salida_manual_actual:
            if self.salida_manual_actual == "RFALLA_MANUAL":
                self.salidas["P1RET"] = self.salidas["P2RET"] = self.salidas["P3RET"] = True
                if self.inicio_manual is None:
                    self.inicio_manual = time.time()
                    print("⚙️ [Manual] Activando P1RET, P2RET y P3RET")
                if self.latch["p1r"] and self.latch["p2r"] and self.latch["p3r"]:
                    self.salida_manual_actual = None
                    self.inicio_manual = None
            else:
                salida = self.salida_manual_actual
                if salida in self.salidas:
                    self.salidas[salida] = True
                    if self.inicio_manual is None:
                        self.inicio_manual = time.time()
                        print(f"⚙️ [Manual] Activando salida: {salida}")
                sensor_obj = {v: k for k, v in SENSOR_SALIDA_MAP.items()}.get(salida)
                if sensor_obj and self.latch[sensor_obj]:
                    self.salida_manual_actual = None
                    self.inicio_manual = None
        if self.inicio_manual and time.time() - self.inicio_manual > 10:
            print("❌ Falla en modo manual: sensor no activado a tiempo.")
            self.fallo = True

    def run_grafcet(self):
        self.salida()
        while self.running:
            if not self.pausa and not self.fallo:
                if self.modo == "manual":
                    self.modo_manual()
                else:
                    self.evo()
                self.actualizar_timer()
                self.verificar_fallos()
                self.salida()
                self.start = self.CLC = self.SE = False
                for sensor in self.latch:
                    if not self.salidas_activa_sensor(sensor) and not (self.modo == "manual" and self.salida_manual_actual):
                        self.latch[sensor] = False
            else:
                for t in self.timers_salida.values():
                    t["inicio"] = None
                    t["terminado"] = False
                self.tiempo_salida = time.time()
            time.sleep(0.2)

    def ciclo(self):
        print("⚙️ Simulador GRAFCET listo.")
        threading.Thread(target=self.run_grafcet, daemon=True).start()
        while self.running:
            cmd = input("\nComandos: start, clc, se, modo, paro, ok, reset, rfalla, sensores, salidas...\nComando: ").strip().upper()
            if cmd == "START":
                self.start = True
            elif cmd == "CLC":
                self.CLC = True
            elif cmd == "SE":
                self.SE = True
            elif cmd.lower() in self.sensores:
                sensor = cmd.lower()
                self.sensores[sensor] = True
                if self.salidas_activa_sensor(sensor):
                    self.latch[sensor] = True
            elif cmd == "MODO":
                nuevo = input("Modo [automatico | ciclo | etapa | manual]: ").strip().lower()
                if nuevo in ["automatico", "ciclo", "etapa", "manual"]:
                    self.modo = nuevo
                    self.start = self.CLC = self.SE = False
                    self.salida_manual_actual = None
                    self.inicio_manual = None
                    print(f"✅ Modo cambiado a {self.modo}")
            elif cmd == "PARO":
                self.pausa = True
                print("🟥 PARO DE EMERGENCIA activado.")
            elif cmd == "OK":
                self.pausa = False
                print("▶️ Proceso reanudado.")
            elif cmd == "RESET":
                self.running = False
                print("🔄 Sistema reiniciado.")
                time.sleep(0.5)
                self.__init__()
                self.ciclo()
                return
            elif cmd == "RFALLA":
                if self.modo == "manual":
                    self.salida_manual_actual = "RFALLA_MANUAL"
                    self.inicio_manual = None
                    print("⚙️ [Manual] Activando P1RET, P2RET y P3RET simultáneamente para prueba de falla.")
                else:
                    self.running = False
                    print("✅ Falla reseteada.")
                    time.sleep(0.5)
                    self.__init__()
                    self.ciclo()
                    return
            elif self.modo == "manual" and cmd in self.salidas:
                self.salida_manual_actual = cmd
                self.inicio_manual = None
                print(f"⚙️ [Manual] Activando salida: {cmd}")
            else:
                print("❓ Comando no reconocido.")

    def salidas_activa_sensor(self, sensor):
        salida = SENSOR_SALIDA_MAP.get(sensor)
        return self.salidas.get(salida, False)

# ========================= INICIO DEL PROGRAMA ========================= #
grafcet = GrafcetSimulador()
grafcet.ciclo()
