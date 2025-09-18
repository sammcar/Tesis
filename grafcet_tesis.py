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
        # Mapa inverso salida -> sensor (si aplica)
        self.sensor_por_salida = {v: k for k, v in SENSOR_SALIDA_MAP.items()}

        # Estado de operación manual en curso
        self.manual_deadline = None   # timestamp límite para la acción manual actual



    # ========================= FUNCIONES INTERNAS ========================= #

    def reset_to_stage0(self):
        """Resetea a etapa 0 manteniendo modo y sin relanzar hilos."""
        # Etapas
        self.etapas = [False] * NUM_ETAPAS
        self.etapas[0] = True
        self.etapa_anterior = -1

        # Estado de proceso
        self.fallo = False
        # NO tocamos self.pausa (paro es solo pausa)
        # start se deja como esté (palanca persistente)

        # Sensores y latches
        for k in self.sensores:
            self.sensores[k] = False
        for k in self.latch:
            self.latch[k] = False

        # Timers de salidas
        for t in self.timers_salida.values():
            t["inicio"] = None
            t["terminado"] = False

        # Timer de falla por etapa
        self.tiempo_salida = time.time()
        self.etapa_timer_fallo = 0

        # Forzar actualización de salidas (imprimirá etapa 0 una vez)
        self.salida()


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
            (3, (not self.colectorEN) and condi, 5),
            (4, s["ok485"] and self.colectorEN and condi, 5),
            (5, (not self.colectorEN) and condiCL, 6),
            (5, self.colectorEN and condiCL, 7),
            (6, (not self.colectorEN) and condi, 8),
            (7, s["vd"] and s["ok485"] and self.colectorEN and condi, 8),
            (8, s["mpd"] and s["spd"] and condi, 9),
            (9, s["son"] and s["okmon"] and condi, 10),
            (10, condiCL, 11),
            (11, s["soff"] and s["mh"] and s["sh"] and (not self.colectorEN) and condi, 12),
            (11, s["soff"] and s["mh"] and s["sh"] and self.colectorEN and condi, 13),
            (12, (not self.colectorEN) and condi, 14),
            (13, s["voff"] and self.colectorEN and condi, 14),
            (14, self.start and condi, 0),
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
            print(f"[ESTADO] START={self.start} | MODO={self.modo}")
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
        """
        En manual:
        - self.salida_manual_actual puede ser una salida normal o 'timer_*'.
        - Si la salida tiene sensor asociado, se espera su latch antes del timeout.
        - Si es 'timer_*', se respeta su duración.
        - 'RFALLA_MANUAL' ejecuta un rescate (SYOFF, MOTORHOME, SERVOHOME).
        """

        # Si hay falla: no permitir salidas manuales
        if self.fallo:
            for k in self.salidas:
                self.salidas[k] = False
            self.salida_manual_actual = None
            self.inicio_manual = None
            self.manual_deadline = None
            return

        # Apaga todo y luego enciende solo lo pedido
        for k in self.salidas:
            self.salidas[k] = False

        salida = self.salida_manual_actual
        if not salida:
            self.inicio_manual = None
            self.manual_deadline = None
            return

        # === Rescate manual ===
        if salida == "RFALLA_MANUAL":
            for out in ("SYOFF", "MOTORHOME", "SERVOHOME"):
                if out in self.salidas:
                    self.salidas[out] = True

            if self.inicio_manual is None:
                self.inicio_manual = time.time()
                self.manual_deadline = self.inicio_manual + 15
                print("⚙️ [Manual] Rescate: SYOFF + MOTORHOME + SERVOHOME")

            # Verifica sensores de las tres
            ok = 0
            for out in ("SYOFF", "MOTORHOME", "SERVOHOME"):
                sensor = self.sensor_por_salida.get(out)
                if sensor and self.latch.get(sensor, False):
                    ok += 1
            if ok == 3:
                print("✅ [Manual] Rescate completado.")
                self.salida_manual_actual = None
                self.inicio_manual = None
                self.manual_deadline = None
                return

            if self.manual_deadline and time.time() > self.manual_deadline:
                print("❌ [Manual] Rescate: sensores no confirmados a tiempo.")
                self.fallo = True
            return

        # === Salidas timer_* ===
        if salida.startswith("timer_"):
            if salida not in self.timers_salida:
                print(f"⚠️ [Manual] {salida} no tiene duración definida en SALIDAS_TIMER_DURACION.")
                self.salida_manual_actual = None
                self.inicio_manual = None
                self.manual_deadline = None
                return

            self.salidas[salida] = True
            timer = self.timers_salida[salida]
            if timer["inicio"] is None:
                timer["inicio"] = time.time()
                timer["terminado"] = False
                print(f"⏱️ [Manual] {salida} iniciado por {timer['duracion']} s")

            if (time.time() - timer["inicio"]) >= timer["duracion"]:
                timer["terminado"] = True
                print(f"✅ [Manual] {salida} finalizado")
                self.salida_manual_actual = None
                self.inicio_manual = None
                self.manual_deadline = None
            return

        # === Salidas normales ===
        self.salidas[salida] = True

        # Si tiene sensor asociado, arma deadline por sensor; si no, un “tic” corto
        if self.inicio_manual is None:
            self.inicio_manual = time.time()
            sensor_obj = self.sensor_por_salida.get(salida)
            if sensor_obj:
                timeout = TIEMPO_FALLA_SENSORES.get(sensor_obj, 10)
                self.manual_deadline = self.inicio_manual + timeout
                print(f"⚙️ [Manual] {salida}: esperando sensor '{sensor_obj}' ≤ {timeout}s")
            else:
                self.manual_deadline = self.inicio_manual + 0.5
                print(f"⚙️ [Manual] {salida}: sin sensor asociado (operación breve)")

        sensor_obj = self.sensor_por_salida.get(salida)
        if sensor_obj and self.latch.get(sensor_obj, False):
            print(f"✅ [Manual] {salida}: sensor '{sensor_obj}' confirmado.")
            self.salida_manual_actual = None
            self.inicio_manual = None
            self.manual_deadline = None
            return

        if self.manual_deadline and time.time() > self.manual_deadline:
            if sensor_obj:
                print(f"❌ [Manual] {salida}: sensor '{sensor_obj}' no confirmado a tiempo.")
                self.fallo = True
                # Limpieza
                self.salida_manual_actual = None
                self.inicio_manual = None
                self.manual_deadline = None
                return
            else:
                print(f"ℹ️ [Manual] {salida}: operación breve completada.")
                self.salida_manual_actual = None
                self.inicio_manual = None
                self.manual_deadline = None

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
                self.CLC = self.SE = False
                for sensor in self.latch:
                    if not self.salidas_activa_sensor(sensor) and not (self.modo == "manual" and self.salida_manual_actual):
                        self.latch[sensor] = False
            else:
                for t in self.timers_salida.values():
                    t["inicio"] = None
                    t["terminado"] = False
                self.tiempo_salida = time.time()
            time.sleep(0.5)  # refresco más lento para no spamear demasiado

    def ciclo(self):
        print("⚙️ Simulador GRAFCET listo.")
        threading.Thread(target=self.run_grafcet, daemon=True).start()
        while self.running:
            cmd = input("\nComandos: start, clc, se, modo, paro, ok, reset, rfalla, sensores, salidas...\nComando: ").strip().upper()
            if cmd == "START":
                self.start = not self.start
                print(f"▶️  START = {self.start}")
            elif cmd == "CLC":
                self.CLC = True
            elif cmd == "SE":
                self.SE = True
            elif cmd.lower() in self.sensores:
                if self.fallo:
                    print("⛔ Sensor ignorado: sistema en FALLA. Use 'rfalla'.")
                    continue
                sensor = cmd.lower()
                self.sensores[sensor] = True
                if self.salidas_activa_sensor(sensor):
                    self.latch[sensor] = True

            elif cmd == "MODO":
                nuevo = input("Modo [automatico | ciclo | etapa | manual]: ").strip().lower()
                if nuevo in ["automatico", "ciclo", "etapa", "manual"]:
                    self.modo = nuevo
                    self.CLC = self.SE = False
                    self.salida_manual_actual = None
                    self.inicio_manual = None
                    self.reset_to_stage0()
                    print(f"✅ Modo cambiado a {self.modo} y proceso reiniciado en etapa 0")
            elif cmd == "PARO":
                self.pausa = True
                print("🟥 PARO DE EMERGENCIA activado.")
            elif cmd == "OK":
                self.pausa = False
                print("▶️ Proceso reanudado.")
            elif cmd == "RESET":
                self.reset_to_stage0()
                print("🔄 Sistema reiniciado.")
                time.sleep(0.5)
            elif cmd == "RFALLA":
                # Limpia falla, cancela acción manual y vuelve a etapa 0
                self.fallo = False
                self.salida_manual_actual = None
                self.inicio_manual = None
                self.manual_deadline = None
                self.reset_to_stage0()
                print("✅ Falla reseteada y proceso en etapa 0.")
            elif self.modo == "manual" and cmd == "STOP":
                self.salida_manual_actual = None
                self.inicio_manual = None
                self.manual_deadline = None
                print("⏹️ [Manual] Acción manual cancelada.")
            elif self.modo == "manual" and cmd in self.salidas:
                if self.fallo:
                    print("⛔ [Manual] Bloqueado por FALLA. Use 'rfalla' para resetear.")
                    continue
                self.salida_manual_actual = cmd
                self.inicio_manual = None
                self.manual_deadline = None
                print(f"⚙️ [Manual] Activando salida: {cmd}")
            else:
                print("❓ Comando no reconocido.")

    def salidas_activa_sensor(self, sensor):
        salida = SENSOR_SALIDA_MAP.get(sensor)
        # En manual, si la salida objetivo coincide, considérela activa (evita carrera)
        if self.modo == "manual" and self.salida_manual_actual == salida:
            return True
        return self.salidas.get(salida, False)

# ========================= INICIO DEL PROGRAMA ========================= #
grafcet = GrafcetSimulador()
grafcet.ciclo()
