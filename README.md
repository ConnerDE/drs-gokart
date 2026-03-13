# DRS System für ein Gokart (ESP32-S3)

## Überblick
Dieses Projekt implementiert ein **DRS-System (Drag Reduction System)** für ein Gokart.  
Die Steuerung erfolgt über einen **ESP32-S3**, der einen Aktuator bzw. Servo ansteuert, um eine verstellbare aerodynamische Komponente (z. B. Heckflügel-Element) zu öffnen oder zu schließen.


## Funktionen
- Steuerung des DRS über den **ESP32-S3**
- Öffnen und Schließen des DRS per Taster am Lenkrad
- Sicherheitslogik, um ungewollte Aktivierung zu verhindern (-> nicht in Kurven, bei Regen oder starkem Abbremsen aktiviertbar)

## Hardware
- **ESP32-S3**
- 2x Stellmotoren
- ...

## Software
Die Software läuft auf dem **ESP32-S3** und übernimmt:
- Einlesen der Eingaben (z. B. Button)
- Steuerung des Servos/Aktuators
- Zustandsverwaltung des DRS (offen / geschlossen)

## Projektstruktur
