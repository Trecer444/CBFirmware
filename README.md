# CBFirmware

**CBFirmware** — прошивка для микроконтроллера **STM32F405RGT6**, предназначенная для управления шестью силовыми выходами на мотоцикле **BMW F800GS**. Устройство принимает команды по **CAN-шине** (например, включение зажигания, света и пр.) и управляет соответствующими нагрузками. Проект поддерживает обновление прошивки по USB с помощью загрузчика **OpenBLT**. Основная логика реализована в `Prog/Core/Src`.

## 📦 Возможности

- Управление 6 силовыми выходами
- Поддержка CAN-команд
- Работа с пользовательскими настройками
- Обновление прошивки через USB
- Конфигурирование через графическую утилиту

## 🔗 Связанные проекты

- ⚙️ Аппаратная часть: [CBHardware](https://github.com/Trecer444/CBHardware)
- 🖥 GUI для настройки: [CBGUISettings](https://github.com/Trecer444/CBGUISettings)
- 🚀 Загрузчик: [OpenBLT](https://github.com/feaser/openblt)

## 🗂 Структура проекта

```
CBFirmware/
├── .ioc                          # Конфигурация CubeMX
├── Core/                         # Автогенерация HAL
├── Drivers/                      # HAL/CMSIS
├── Prog/
│   └── Core/
│       ├── Src/ (main, settings, outputch)
│       └── Inc/
├── Boot/                         # Проект загрузчика OpenBLT
├── Host/                         # BootCommander.exe для USB-прошивки
```

## 🛠 Сборка и прошивка

### Требования

- [STM32CubeIDE v1.18.1](https://www.st.com/en/development-tools/stm32cubeide.html)
- ST-Link для первичной прошивки
- USB для последующих обновлений через загрузчик

### Сборка проекта

1. Клонировать проект:
   ```bash
   git clone https://github.com/Trecer444/CBFirmware.git
   ```
2. Открыть оба проекта в STM32CubeIDE (`CBFirmware` и `Boot`)
3. Собрать проекты
4. Первично прошить загрузчик (`Boot`) через ST-Link
5. Далее использовать `Host/BootCommander.exe` для обновлений через USB

## 🔌 USB-прошивка (OpenBLT)

1. Подключите плату по USB
2. Запустите `BootCommander.exe` из папки `Host`
3. Выберите `.hex` прошивку и нужный COM-порт
4. Нажмите "Start" для загрузки

## 📜 Лицензия

Этот проект распространяется под лицензией **GNU GPLv3**. Подробнее см. файл [LICENSE](./LICENSE).
