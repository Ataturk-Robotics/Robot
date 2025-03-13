# ROBOT 2025

[![CI](https://github.com/Ataturk-Robotics/Robot/actions/workflows/ci.yml/badge.svg)](https://github.com/Ataturk-Robotics/Robot/actions/workflows/ci.yml)

10185 Hydrob'un *2025* FRC robot kodu. 

The code for the team 10185 Hydrob *2025* FRC robot code.

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
git clone https://github.com/Ataturk-Robotics/Robot/tree/Robot-2023
```
