#!/usr/bin/env python3

import asyncio

class Mission:
    """
    Tüm görev sınıfları için temel sınıf.
    Her özel görev sınıfı bu sınıftan türemeli ve 'start' metodunu implemente etmelidir.
    """
    def __init__(self, drone_controller, params: dict):
        self.drone_controller = drone_controller
        self.params = params
        self.mission_id = params.get("mission_id", "UnknownMission")
        self.all_confirmed_event = asyncio.Event() # Görev onayları için event

    async def start(self):
        """
        Görevin ana mantığını içeren asenkron metot.
        Bu metot, türetilmiş sınıflar tarafından implemente edilmelidir.
        """
        raise NotImplementedError("Her görev sınıfı 'start' metodunu implemente etmelidir.")

    async def wait_for_all_confirmations(self, timeout: float = 30.0):
        """
        Diğer dronlardan görev onaylarını bekler.
        """
        print(f"[Mission {self.mission_id}]: Diğer dronlardan onay bekleniyor...")
        try:
            await asyncio.wait_for(self.all_confirmed_event.wait(), timeout=timeout)
            print(f"[Mission {self.mission_id}]: Tüm onaylar alındı.")
            return True
        except asyncio.TimeoutError:
            print(f"[Mission {self.mission_id}]: Onaylar için zaman aşımı oluştu.")
            return False

