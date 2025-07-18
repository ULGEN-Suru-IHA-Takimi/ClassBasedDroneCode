#!/usr/bin/env python3

class waypoints:
    def __init__(self):
        self.list={}
    def add(self,id,lat,lon,alt,hed):
        self.list[id]=Waypoint(lat,lon,alt,hed)
        print(f"    Waypoint eklendi/güncellendi: id={id}, latitude={lat}, longitude={lon}, altitude={alt}, heading={hed}")

    def read(self,id):
        try:
            return self.list[id] 
        except KeyError: 
            print(f"    Waypoint okuma başarısız: id={id} bulunamadı.")
            return None
        except Exception as e: 
            print(f"    Waypoint okuma sırasında beklenmedik hata: {e}")
            return None

    def remove(self,id):
        try:
            self.list.pop(id)
            print(f"    Waypoint silindi: id={id}")
        except KeyError:
            print(f"    Waypoint silme başarısız: id={id} bulunamadı.")
        except Exception as e:
            print(f"    Waypoint silme sırasında beklenmedik hata: {e}")


class Waypoint: # Sınıf adı Waypoint
    def __init__(self,lat,lon,alt,hed):
        self.lat = lat
        self.lon = lon
        self.alt = alt
        self.hed = hed

if __name__ == "__main__":
    pass