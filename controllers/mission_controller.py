#!/usr/bin/env python3

import asyncio
from abc import ABC, abstractmethod

class Mission(ABC):
    """
    Tüm görev sınıfları için temel sınıf.
    Her özel görev sınıfı bu sınıftan türemeli ve 'start' metodunu implemente etmelidir.
    """
    # MISSION_ID ve mission_name artık burada tanımlanmayacak, türetilmiş sınıflar tarafından sağlanacak.

    def __init__(self, drone_controller, mission_id: str, mission_name: str, params: dict):
        self.drone_controller = drone_controller
        self.mission_id = mission_id
        self.mission_name = mission_name
        self.params = params
        self.all_confirmed_event = asyncio.Event()

    @abstractmethod
    async def start(self):
        """
        Görevin ana mantığını içeren asenkron metot.
        Bu metot, türetilmiş sınıflar tarafından implemente edilmelidir.
        """
        pass

    async def wait_for_all_confirmations(self, timeout: float = 30.0):
        """
        Diğer dronlardan görev onaylarını bekler.
        """
        print(f"[Mission {self.mission_id} - {self.mission_name}]: Diğer dronlardan onay bekleniyor...")
        try:
            await asyncio.wait_for(self.all_confirmed_event.wait(), timeout=timeout)
            print(f"[Mission {self.mission_id} - {self.mission_name}]: Tüm onaylar alındı.")
            return True
        except asyncio.TimeoutError:
            print(f"[Mission {self.mission_id} - {self.mission_name}]: Onaylar için zaman aşımı oluştu.")
            return False

