import time
import math


# Variables internes
_last_checkpoint = 0.0      # distance accumulée depuis le dernier déclenchement
_last_trigger_time = time.time()  # temps du dernier déclenchement

# Constantes pour conversion ticks → mètres
WHEEL_DIAMETER = 0.15       # m
ENC_RESOLUTION = 131072     # ticks/tour
TICKS_TO_M = (math.pi * WHEEL_DIAMETER) / ENC_RESOLUTION

# Empattement du robot (distance entre roues gauche et droite)
WHEEL_BASE = 0.4  # à ajuster selon ton robot

# Mémoire pour les encodeurs
encoders = {
    "left": {"last_ticks": None, "id": 0x601},
    "right": {"last_ticks": None, "id": 0x602}
}


def decode_ticks(data_bytes):
    """
    Décode 4 octets little endian (signed) en ticks.
    """
    return int.from_bytes(data_bytes[0:4], byteorder="little", signed=True)


# Dictionnaire encodeurs
encoders = {
    "right": {"id": 0x381, "last_ticks": None, "delta_m": None},
    "left": {"id": 0x382, "last_ticks": None, "delta_m": None},
}

def process_encoder(can_id, data_bytes):
    """
    Met à jour les encodeurs et renvoie (delta_d, delta_theta)
    uniquement quand les deux roues ont donné un delta.
    """
    ticks = decode_ticks(data_bytes)

    # Identifier la roue en fonction de l'ID
    for wheel, info in encoders.items():
        if can_id == info["id"]:
            if info["last_ticks"] is None:
                # Première mesure -> juste mémoriser
                info["last_ticks"] = ticks
                return None

            # Calcul delta ticks -> distance
            delta_ticks = ticks - info["last_ticks"]
            delta_m = delta_ticks * TICKS_TO_M
            info["last_ticks"] = ticks
            info["delta_m"] = delta_m
            break
    else:
        # ID inconnu
        return None

    # Vérifier si on a reçu les deux deltas
    if encoders["left"].get("delta_m") is not None and encoders["right"].get("delta_m") is not None:
        dl = encoders["left"]["delta_m"]
        dr = encoders["right"]["delta_m"]

        # Réinitialiser pour la prochaine mesure
        encoders["left"]["delta_m"] = None
        encoders["right"]["delta_m"] = None

        # Calcul odométrie
        delta_d = (dr + dl) / 2.0
        delta_theta = (dr - dl) / WHEEL_BASE  # WHEEL_BASE = écart entre roues en mètres

        return delta_d, delta_theta

    # Pas encore les deux roues -> rien à renvoyer
    return None


    # Vérifier si les deux roues ont un delta dispo
    if "delta_m" in encoders["left"] and "delta_m" in encoders["right"]:
        dl = encoders["left"].pop("delta_m")
        dr = encoders["right"].pop("delta_m")

        # Odométrie différentielle
        delta_d = (dr + dl) / 2.0
        delta_theta = (dr - dl) / WHEEL_BASE
        return delta_d, delta_theta

    return None

def check_distance_or_time(delta_d, threshold_dist=0.3, threshold_time=9.0):
    """
    Déclenche si :
      - le robot a parcouru >= threshold_dist mètres depuis le dernier déclenchement
      - OU si threshold_time secondes se sont écoulées
    Renvoie True si un déclenchement doit avoir lieu, False sinon.
    """
    global _last_checkpoint, _last_trigger_time

    # Accumule la distance absolue
    _last_checkpoint += abs(delta_d)

    now = time.time()
    elapsed = now - _last_trigger_time

    if _last_checkpoint >= threshold_dist or elapsed >= threshold_time:
        # reset pour le prochain cycle
        _last_checkpoint = 0.0
        _last_trigger_time = now
        return True

    return False
