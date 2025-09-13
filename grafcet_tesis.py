import threading
import time

# ========================= CONFIGURACIÓN EXTERNA ========================= #

NUM_ETAPAS = 15
SENSORES = ["sh", "mh", "ok232", "ok485", "fcl", "vd", "spd", "mpd", "son", "okmon", "soff", "voff"]
SALIDAS = ["SERVOHOME", "MOTORHOME", "P232", "P485", "OKCL", "VACON", "SERVOMOVE", "MOTORMOVE", "SYON", "MON", "SYOFF", "VACOFF"]

# Asociación sensor → salida
SENSOR_SALIDA_MAP = {
    "sh": "SERVOHOME", "ok485": "P485", "spd": "SERVOMOVE", "okmon": "MON",
    "mh": "MOTORHOME", "fcl": "OKCL", "mpd": "MOTORMOVE", "soff": "SYOFF",
    "ok232": "P232", "vd": "VACON", "son": "SYON", "voff": "VACOFF",
}

# Asociación etapa → salidas
ETAPA_SALIDAS_MAP = {
    0: [],
    1: ["SERVOHOME", "MOTORHOME", "SYOFF", "VACOFF"],
    2: ["P232"],
    3: [],
    4: ["P485"],
    5: ["OKCL"],
    6: [],
    7: ["VACON"],
    8: ["SERVOMOVE", "MOTORMOVE"],
    9: ["SYON","MON"],
    10: ["OKCL"],
    11: ["SERVOHOME","MOTORHOME", "SYOFF"],
    12: [],
    13: ["VACOFF"],
    14: ["OKCL"],
}

# Funciones asociadas a salidas (opcional)
def prender_led(color):
    print(f"[FUNC] LED {color.upper()} encendido")

SALIDA_FUNCIONES = {
    "OKCL": lambda: prender_led("verde"),
}

# Duraciones de temporizadores por salida tipo "timer_*"
SALIDAS_TIMER_DURACION = {
    #"timer_5": 5,
    #"timer_3": 3,
}

# Tiempos de falla por sensor (segundos)
TIEMPO_FALLA_SENSORES = {
    "sh": 20, "ok485": 5, "spd": 20, "okmon": 9999,
    "mh": 20, "fcl": 9999, "mpd": 20, "soff": 15,
    "ok232": 5, "vd": 20, "son": 15, "voff": 20,
}

# Etapas que esperan sensores
ETAPA_SENSORES_FALLO = {
    0: [],
    1: ["sh", "mh", "soff", "voff"],
    2: ["ok232"],
    3: [],
    4: ["ok485"],
    5: ["fcl"],
    6: [],
    7: ["vd"],
    8: ["spd", "mpd"],
    9: ["son","okmon"],
    10: ["fcl"],
    11: ["sh","mh", "soff"],
    12: [],
    13: ["voff"],
    14: ["fcl"],
}

# ========================= CLASE GRAFCET ========================= #

class GrafcetSimulador:
    def __init__(self):
        # Estado de etapas
        self.etapas = [False] * NUM_ETAPAS
        self.etapas[0] = True
        self.etapa_anterior = -1
        self.colectorEN = False

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

        condiCL = (automatico or (ciclo and self.CLC) or (etapa and self.SE))
        condi = (automatico or ciclo or (etapa and self.SE))

        transiciones = [
            (0, self.start and condiCL, 1),
            (1, s["sh"] and s["mh"] and s["soff"] and s["voff"] and condi, 2),
            (2, s["ok232"] and (not self.colectorEN) and condi, 3),
            (2, s["ok232"] and self.colectorEN and condi, 4),
            (3, (not self.colectorEN) and condi, 4),
            (4, s["ok485"] and self.colectorEN and condi, 5),
            (5, (not self.colectorEN) and condiCL, 6),
            (5, self.colectorEN and condiCL, 7),
            (6, (not self.colectorEN) and condi, 8),
            (7, s["vd"] and s["ok485"] and self.colectorEN and condi, 8),
            (8, s["mpd"] and s["spd"] and condi, 9),
            (9, s["ok232"] and s["son"] and s["okmon"] and condi, 10),
            (10, s["okmon"] and condiCL, 11),
            (11, s["soff"] and s["mh"] and s["sh"] and (not self.colectorEN) and condi, 12),
            (11, s["soff"] and s["mh"] and s["sh"] and self.colectorEN and condi, 13),
            (12, (not self.colectorEN) and condi, 14),
            (13, s["ok485"] and s["voff"] and self.colectorEN and condi, 14),
            (14, self.start and condiCL, 0),
        ]

        for actual, condicion, nueva in transiciones:
            if E[actual] and condicion:
                self.etapas = [False] * NUM_ETAPAS
                self.etapas[nueva] = True
                return

    def salida(self):
        actual = next((i for i, e in enumerate(self.etapas) if e), -1)

        # Inicializa almacenamiento de estado previo de salidas (una sola vez)
        if not hasattr(self, "_salidas_prev"):
            self._salidas_prev = {k: False for k in self.salidas}

        # Construye el set de salidas que deben estar activas en la etapa actual
        salidas_etapa = set(ETAPA_SALIDAS_MAP.get(actual, []))

        # Actualiza el diccionario de salidas sin imprimir aún
        for k in self.salidas:
            self.salidas[k] = (k in salidas_etapa)

        # Imprime solo cuando cambia la etapa
        if actual != self.etapa_anterior:
            print(f"\n🔁 Etapa activa: {actual}")
            self.etapa_anterior = actual
            # En cambio de etapa, imprime todas las salidas activas una sola vez
            for s in salidas_etapa:
                print(f"⚙️ Activando salida: {s}")
                if s in SALIDA_FUNCIONES:
                    try:
                        SALIDA_FUNCIONES[s]()
                    except Exception as e:
                        print(f"⚠️ Error al ejecutar función de salida {s}: {e}")
        else:
            # Si no cambió de etapa, imprime solo los flancos (cuando una salida cambie de False→True)
            for s in salidas_etapa:
                if not self._salidas_prev.get(s, False) and self.salidas[s]:
                    print(f"⚙️ Activando salida: {s}")
                    if s in SALIDA_FUNCIONES:
                        try:
                            SALIDA_FUNCIONES[s]()
                        except Exception as e:
                            print(f"⚠️ Error al ejecutar función de salida {s}: {e}")

        # Guarda el estado actual para detectar flancos la próxima vez
        self._salidas_prev = self.salidas.copy()


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
