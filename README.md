# ROBOT 2023

[![CI](https://github.com/Ataturk-Robotics/Robot/actions/workflows/ci.yml/badge.svg)](https://github.com/Ataturk-Robotics/Robot/actions/workflows/ci.yml)

8240 Atatürk Robotics'in *2023* FRC robot kodu. 

The code for the team 8240 Atatürk Robotics' *2023* FRC robot code.

### <a href="https://docs.wpilib.org/tr/">WPILIB dokümanları</a>

## Proje yapısı

```
├───src
│   └───main
│       ├───deploy
│       └───java
│           └───frc
│               └───robot
│                   ├───commands
│                   │   ├───ArmCommand       #Kol Mekanizmasını kaldırıp indirme komutları
│                   │   ├───DriveCommand     #Hareket komutları
│                   │   ├───IntakeCommand    #Küp/Koni alma komutları
│                   │   ├───LinearCommand    #Kola açı verme komutları
|                   |   ├───PneumaticCommand #Intake'teki pneumatic sistemini çalıştırma komutları
|                   |   └───TurretCommand    #Turreti döndürme komutları
│                   └───subsystems     #Subsystemler
└───vendordeps                         #Kütüphaneler
```

## Projeyi bilgisayara kopyalama

- <a href="https://git-scm.com">Git</a>'in son sürümünü indirip kurun
- <a href="https://docs.wpilib.org/tr/latest/docs/zero-to-robot/step-2/wpilib-setup.html">WPILIB</a>'un son sürümünü indirip kurun 
- <a href="https://www.ni.com/en-tr/support/downloads/drivers/download.frc-game-tools.html#440024">FRC Game Tools</a>'un son sürümünü indirip kurun *(Sadece kod robota atılacaksa gerekli, simülasyon kullanılacaksa gerek yok)*

```
git clone https://github.com/Ataturk-Robotics/Robot/tree/Robot-2022
```