#include "../config.h"
#include <Arduino.h>
#include <ESPAsyncWebServer.h>  // асинхронний веб-сервер для обробки HTTP-запитів без блокування основного потоку
#include <WiFi.h>
/*
  Асинхронний веб-сервер — це сервер, який обробляє HTTP-запити без блокування
  основного потоку виконання програми. На відміну від синхронного сервера, який
  чекає завершення обробки одного запиту перед початком наступного, асинхронний
  сервер дозволяє обробляти кілька запитів одночасно.

  Як працює:
  Використовує подієво-орієнтований підхід: коли надходить запит, сервер
  реєструє його і повертається до інших завдань, а обробка відбувається "у
  фоновому режимі". У нас це реалізовано через функції зворотного виклику
  (callbacks), які викликаються при запитах на /mode1, /mode2, /mode3.

  Переваги:
    1) ефективність: Не блокує ядро, дозволяючи виконувати інші задачі
  (наприклад, обробку звуку). 2) швидкість: швидше реагує на кілька запитів, що
  важливо для веб-інтерфейсу. 3) масштабованість: підходить для систем із
  багатьма клієнтами. Це дозволяє ESP32 одночасно слухати запити і керувати
  світлодіодами без затримок.
*/

#include <FastLED.h>  // бібліотека для керування адресними світлодіодами (наприклад, WS2812B)

#include "arduinoFFT.h"  // бібліотека для виконання швидкого перетворення Фур'є (FFT), - аналізуємо звукові частоти

#define SAMPLES 128                // кількість зразків для FFT (128 точок даних для аналізу сигналу)
#define SAMPLING_FREQ 10000        // частота дискретизації (10 кГц), тобто 10 000 зразків за секунду
#define LED_PIN_CIRCLES 25         // пін для кіл (16 + 12 LED)
#define LED_PIN_SQUARES 26         // пін для квадратів (16 + 16 LED)
#define NUM_LEDS_CIRCLE1 16        // перше коло
#define NUM_LEDS_CIRCLE2 12        // друге коло
#define NUM_LEDS_SQUARE1 16        // перший квадрат
#define NUM_LEDS_SQUARE2 16        // другий квадрат
#define NUM_LEDS_CIRCLES_TOTAL 28  // усі світлодіоди кіл
#define NUM_LEDS_SQUARES_TOTAL 32  // усі світлодіоди квадратів

const char *ssid = WIFI_SSID;
const char *password = WIFI_PASSWORD;

CRGB leds_circles[NUM_LEDS_CIRCLES_TOTAL];      // масив для кіл (16 + 12)
CRGB leds_squares[NUM_LEDS_SQUARES_TOTAL];      // масив для квадратів (16 + 16)
AsyncWebServer server(80);                      // об’єкт асинхронного веб-сервера, що слухає порт 80 (стандартний HTTP-порт)
ArduinoFFT<double> FFT = ArduinoFFT<double>();  // об’єкт FFT для аналізу сигналу
double vReal[SAMPLES];                          // масив для зберігання реальних частин сигналу
double vImag[SAMPLES];                          // масив для зберігання уявних частин сигналу
volatile int mode = 3;                          // поточний режим роботи (встановлюється віддалено через веб-сервер);
                                                // volatile, бо використовується у кількох задачах

int ampR, ampG, ampB;  // для зберігання амплітуд частотних діапазонів: басів (R), середніх частот (G), високих частот (B)

// У скетчі використовуємо багатозадачність FreeRTOS, розподіляючи на різні ядра
// ESP32-WROOM-32D роботу веб-сервера (ядро 0) і обробку звуку/світла (ядро 1)
/*
  FreeRTOS (Free Real-Time Operating System) — це безкоштовна операційна система
  реального часу з відкритим кодом, призначена для вбудованих систем, таких як
  мікроконтролери (наприклад, ESP32). Вона дозволяє запускати кілька завдань
  (tasks) одночасно, імітуючи багатозадачність на пристроях з обмеженими
  ресурсами.

  FreeRTOS має вбудований планувальник, який визначає, яке завдання виконувати в
  конкретний момент, базуючись на їх пріоритетах. Кожне завдання (tasks) — це
  окрема функція, яка виконується "паралельно" з іншими. На нашому двоядерному
  процесорі можна "прив’язати" завдання до конкретного ядра (core 0 або core 1).
  Завдання мають пріоритети (чим вище число, тим вищий пріоритет), що впливає на
  порядок їх виконання. FreeRTOS швидко перемикається між завданнями, створюючи
  ілюзію одночасної роботи.

  Усе це дозволяє розділяти логіку програми на незалежні блоки (у нас -- на
  веб-сервер і обробку звуку). Покращує використання ресурсів мікроконтролера.
  Забезпечує реакцію в реальному часі (це важливо для світломузики).
*/

void webServerTask(void *pvParameters) {  // завдання для веб-сервера, що працює на ядрі 0
  /*
    Використовуємо Callback (зворотний виклик) — це функція, яка передається як
    аргумент іншій функції і викликається пізніше, коли настає певна подія. У
    нас callbacks використовуються для обробки HTTP-запитів асинхронним
    веб-сервером. Замість того, щоб постійно перевіряти, чи надійшов запит, ми
    "реєструємо" функцію (callback), яка автоматично викликається сервером, коли
    подія (наприклад, GET-запит) відбувається. Це ключова частина асинхронного
    програмування: код не блокується в очікуванні, а реагує на події.

    Нижче [](AsyncWebServerRequest *request) { ... } — це callback.
    Він передається методу server.on і викликається, коли клієнт надсилає
    GET-запит на /mode1. Усередині callback: mode = 1 змінює режим.
      request->send(...) відправляє відповідь клієнту.
    Бібліотека ESPAsyncWebServer працює на основі подій.
    Коли надходить запит, вона викликає зареєстрований callback, передаючи йому
    об’єкт request із деталями запиту. У нас є три callbacks - для /mode1,
    /mode2, /mode3. Переваги: не блокуємо ядро 0, дозволяючи йому виконувати
    інші задачі (наприклад, цикл у webServerTask). Без "ручного" опитування і
    швидко реагуємо на запити.
  */

  server.on("/mode1", HTTP_GET, [](AsyncWebServerRequest *request) {
    mode = 1;
    AsyncWebServerResponse *response = request->beginResponse(200, "text/plain", "OK");
    response->addHeader("Access-Control-Allow-Origin",
                        "*");  // Додаємо CORS-заголовок
    request->send(response);
  });
  server.on("/mode2", HTTP_GET, [](AsyncWebServerRequest *request) {
    mode = 2;
    AsyncWebServerResponse *response = request->beginResponse(200, "text/plain", "OK");
    response->addHeader("Access-Control-Allow-Origin", "*");
    request->send(response);
  });
  server.on("/mode3", HTTP_GET, [](AsyncWebServerRequest *request) {
    mode = 3;
    AsyncWebServerResponse *response = request->beginResponse(200, "text/plain", "OK");
    response->addHeader("Access-Control-Allow-Origin", "*");
    request->send(response);
  });
  server.on("/mode4", HTTP_GET, [](AsyncWebServerRequest *request) {
    mode = 4;
    AsyncWebServerResponse *response = request->beginResponse(200, "text/plain", "OK");
    response->addHeader("Access-Control-Allow-Origin", "*");
    request->send(response);
  });
  server.on("/mode5", HTTP_GET, [](AsyncWebServerRequest *request) {
    mode = 5;
    AsyncWebServerResponse *response = request->beginResponse(200, "text/plain", "OK");
    response->addHeader("Access-Control-Allow-Origin", "*");
    request->send(response);
  });
  server.on("/mode6", HTTP_GET, [](AsyncWebServerRequest *request) {
    mode = 6;
    AsyncWebServerResponse *response = request->beginResponse(200, "text/plain", "OK");
    response->addHeader("Access-Control-Allow-Origin", "*");
    request->send(response);
  });
  server.on("/mode7", HTTP_GET, [](AsyncWebServerRequest *request) {
    mode = 7;
    AsyncWebServerResponse *response = request->beginResponse(200, "text/plain", "OK");
    response->addHeader("Access-Control-Allow-Origin", "*");
    request->send(response);
  });
  /*
    Параметри:
      "/mode1" — це шлях (URL), на який сервер реагує. Якщо клієнт надсилає
    запит на http://<IP-адреса>/mode1, спрацьовує ця функція. Може бути
    будь-яким рядком, наприклад, "/status" або "/toggle".

      HTTP_GET — тип HTTP-запиту. Тут це GET-запит (клієнт запитує дані).
      Інші можливі значення: HTTP_POST, HTTP_PUT, HTTP_DELETE тощо.

      *[](AsyncWebServerRequest request) { ... } — це лямбда-функція (анонімна
    функція), яка визначає, що робити при запиті. AsyncWebServerRequest *request
    — вказівник на об’єкт запиту, через який можна отримати параметри або
    відправити відповідь. У тілі: mode = 1 змінює режим, а request->send(200,
    "text/plain", "OK") відправляє відповідь: 200 — код статусу (OK).
          "text/plain" — тип вмісту (MIME-тип), може бути "application/json",
    "text/html" тощо. "OK" — текст відповіді, може бути будь-яким рядком. Дужки:
      Круглі дужки (): Визначають аргументи методу server.on. Усі три параметри
    розділені комами. Квадратні дужки []: Частина синтаксису лямбда-функції в
    C++. Порожні [] означають, що зовнішні змінні не захоплюються. Якщо б треба
    було використати зовнішню змінну, писали б, наприклад, [mode]. Фігурні дужки
    {}: Тіло лямбда-функції, де описана логіка.

    Лямбда-функція — це безіменна (анонімна) функція, яка створюється "на льоту"
    прямо в коді. Вона введена у стандарті C++11 і дозволяє писати компактний
    код для одноразових задач, таких як callbacks. Синтаксис:
    [capture](parameters) { body } [capture] — визначає, які зовнішні змінні
    доступні всередині лямбди:
      [] — нічого не захоплює.
      [=] — захоплює все за значенням.
      [&] — захоплює все за посиланням.
      [x, &y] — конкретні змінні (x за значенням, y за посиланням).
      (parameters) — аргументи функції, як у звичайних функціях.
      { body } — код, який виконується.

    У нас:
      [] — порожнє захоплення, бо лямбда використовує лише глобальну змінну mode
    і параметр request. (AsyncWebServerRequest *request) — приймає вказівник на
    об’єкт запиту. { ... } — змінює mode і відправляє відповідь. Призначення: 1)
    компактність: Замінює створення окремої іменованої функції для одноразового
    використання. 2) контекст: Дозволяє легко використовувати локальні змінні
    (через capture). 3) Callbacks: Ідеально підходить для асинхронних подій, як
    у твоєму веб-сервері.

    Приклад:
      int x = 5;
      auto lambda = [x]() { Serial.println(x); }; // Захоплює x за значенням
      lambda(); // Виведе 5
  */

  server.begin();  // запускаємо веб-сервер
  Serial.println("HTTP-сервер запущено на ядрі 0!");
  for (;;) vTaskDelay(10 / portTICK_PERIOD_MS);
  /*
    vTaskDelay - це функція FreeRTOS, яка призупиняє виконання поточного
    завдання на певний час, дозволяючи іншим завданням працювати. Параметр: 10 /
    portTICK_PERIOD_MS — час затримки в "тиках" (ticks). portTICK_PERIOD_MS — це
    константа, яка визначає, скільки мілісекунд у одному тику (зазвичай 1 мс на
    ESP32). Тобто тут затримка = 10 мс. У webServerTask цей цикл потрібен, щоб
    завдання не завершувалося після запуску сервера. Без нього функція б вийшла,
    і завдання зупинилося б. vTaskDelay(10) "звільняє" ядро 0 на 10 мс,
    дозволяючи планувальнику FreeRTOS переключитися на інші завдання (якщо вони
    є) або на фонові процеси ESP32 (наприклад, обробку Wi-Fi). Код поза циклом
    (наприклад, обробка запитів) виконується асинхронно через callbacks, тому
    ядро може їх обробляти, коли цикл "спить".
  */
}

void lightMusicTask(void *pvParameters) {  // обробка звуку та керування світлодіодами, працює на ядрі 1.
  /*
    Логіка роботи: аналіз звуку з аналогового входу, обробка сигналу за
    допомогою FFT для виділення частотних компонентів (баси, середні, високі).
    Керування світлодіодами WS2812B через бібліотеку FastLED у режимах, які
    перемикаються через веб-сервер. Кроки перетворення "сирих" значень у плавну
    світломузику: 
      1) Збір зразків: зчитування 128 значень з мікрофона (analogRead). 
      2) Корекція аномалій: заміна значень < 0 або > 4095 на попереднє або 2048. 
      3) Видалення DC: віднімання середнього для усунення постійної складової. 
      4) Фільтрація: застосування IIR-фільтра для згладжування. 
      5) FFT: перетворення в частотну область (windowing, compute).
      6) Обчислення амплітуд: перехід до величин (complexToMagnitude).
      7) Розподіл частот: поділ на баси, середні, високі.
      8) Нормалізація: масштабування амплітуд за енергією сигналу.
      9) Ковзне середнє: згладжування амплітуд із часом.
      10) Керування LED: переведення амплітуд у кольори/яскравість залежно від
    режиму.
  */

  static double avgAmpR = 0, avgAmpG = 0,
                avgAmpB = 0;  // ці змінні зберігають середні амплітуди між
                              // ітераціями функції lightMusicTask
  /*
    Звичайна локальна змінна "забувається" після завершення функції. Статична
    зберігається в пам’яті і доступна при наступному виклику. Ініціалізація (=
    0) відбувається лише при першому запуску функції. Переваги: 1) Збереження
    стану: дозволяє накопичувати дані (наприклад, середні значення) без
    використання глобальних змінних. 2) Обмежена видимість: доступні лише
    всередині функції, що покращує інкапсуляцію. 3) Ефективність: не потребують
    повторної ініціалізації.
  */
  static int count = 0;

  for (;;) {                             // безкінечний цикл обробки звуку та оновлення світлодіодів
    for (int i = 0; i < SAMPLES; i++) {  // 1) Збір зразків: зчитування 128 значень з мікрофона. +робимо
                                         // перевірку на аномалії за межами діапазону АЦП ESP32 (0–4095)
      vReal[i] = analogRead(34);         // зчитуємо зразки із аналогового входу
      vImag[i] = 0;                      // уявна частина сигналу не потрібна для реального входу
      if (vReal[i] < 0 || vReal[i] > 4095)
        vReal[i] = (i > 0) ? vReal[i - 1] : 2048;  // 2) Корекція аномалій: заміна значень <0
                                                   // або >4095 на попереднє або 2048
      /*
        Чому значення може бути поза межами 0 - 4095:
          1) шум - мікрофон або АЦП можуть видавати аномальні значення через
        електричні перешкоди. 2) помилки АЦП - апаратні збої можуть призводити
        до некоректних даних. Теоретично analogRead не повинен повертати < 0 або
        > 4095, але код додає захист від таких випадків.

        Використовуємо тернарний оператор (?:) :
          (i > 0) — умова: якщо це не перший зразок.
          vReal[i-1] — якщо умова істинна, береться попереднє значення.
          2048 — якщо умова хибна (перший зразок), використовується середнє
        значення діапазону АЦП (4096/2).
      */
      delayMicroseconds(1000000 / SAMPLING_FREQ);
    }

    double mean = 0;  // 3) Видалення DC: віднімання середнього для усунення постійної
                      // складової. Починаємо з підрахунку середнього значення сигналу
    for (int i = 0; i < SAMPLES; i++) mean += vReal[i];
    mean /= SAMPLES;
    for (int i = 0; i < SAMPLES; i++)
      vReal[i] -= mean;  // віднімаємо середнє значення сигналу (mean) від кожного
                         // зразка, щоб позбутися постійної складової (DC offset)

    static unsigned long lastPrint = 0;  // виводимо "сирі" дані з мікрофона (після видалення DC); static -
                                         // щоб змінна зберігала значення між викликами функції
    if (millis() - lastPrint >= 5000) {
      Serial.println("Сигнал із мікрофона (сирі дані, після видалення DC):");
      for (int i = 0; i < SAMPLES; i++) {
        Serial.print(vReal[i]);
        Serial.print(" ");
        if ((i + 1) % 16 == 0) Serial.println();  // розділяємо на групи по 16 значень
      }
      Serial.println();
    }

    double filtered[SAMPLES];  // 4) Фільтрація: застосування IIR-фільтра для згладжування - простий рекурсивний фільтр
                               // сигналу (IIR) щоб згладити сигнал
    for (int i = 0; i < SAMPLES; i++) {
      filtered[i] = vReal[i];
      if (i > 0) filtered[i] = 0.7 * filtered[i - 1] + 0.3 * vReal[i];  // IIR-фільтр: 70% попереднього значення + 30% поточного
    }
    for (int i = 0; i < SAMPLES; i++) vReal[i] = filtered[i];
    /*
      IIR — Infinite Impulse Response (нескінченна імпульсна характеристика) —
      тип цифрового фільтра, який використовує попередні вихідні значення для
      обчислення нового. Це рекурсивний фільтр: 70% попереднього значення + 30%
      поточного. Він згладжує сигнал, зменшуючи різкі стрибки. На відміну від
      FIR (Finite Impulse Response), який працює лише з вхідними даними, IIR
      "пам’ятає" попередні результати, що робить його ефективнішим для
      згладжування. Коефіцієнти (0.7 і 0.3) визначають "силу" згладжування (сума
      = 1 для стабільності). Переваги: простота реалізації і низьке споживання
      ресурсів.
    */

    // 5) FFT: перетворення в частотну область (windowing, compute) - виконуємо
    // послідовно три процедури Fast Fourier Transform, FFT:
    FFT.windowing(vReal, SAMPLES, FFT_WIN_TYP_HAMMING, FFT_FORWARD);  // Функція для зменшення впливу країв сигналу
    /*
      Коли ми беремо скінченний набір зразків сигналу (наприклад, 128 зразків із
      частотою 10 кГц, як у нас), ми фактично "вирізаємо" шматок із
      безперервного сигналу. Цей процес обрізання називається truncation, і він
      створює проблему: краї обрізаного сигналу (початок і кінець) стають
      різкими перепадами (discontinuities), навіть якщо оригінальний сигнал був
      плавним (наприклад, синусоїда). Ці різкі перепади призводять до появи
      спектральних витоків (spectral leakage) у частотній області після FFT.
      Спектральні витоки — це коли енергія однієї частоти "розмазується" на
      сусідні частоти, спотворюючи результат аналізу спектра. Віконне зважування
      вирішує цю проблему, згладжуючи краї сигналу перед FFT: 1) множить кожен
      зразок сигналу на певну вагу, яка залежить від його позиції у наборі
      даних. 2) зазвичай ваги на краях (біля 0 і 127, як у нас) близькі до 0, а
      в центрі (біля 64) — максимальні.

      Використовуємо одну з популярних віконних функцій - вікно Хеммінга
      (FFT_WIN_TYP_HAMMING). До віконування масив vReal містить "сирі" зразки з
      мікрофона, наприклад: [100, 102, 105, ..., 98]. Після віконування кожен
      елемент vReal[i] множиться на відповідне значення вікна Хеммінга: vReal[0]
      = vReal[0] * 0.08 (зменшується). vReal[63] = vReal[63] * 1.0 (залишається
      майже без змін). vReal[127] = vReal[127] * 0.08 (зменшується). Результат:
        Сигнал стає плавнішим на краях, що зменшує різкі перепади.
        Зменшується вплив країв сигналу - без віконування обрізаний сигнал
      виглядає як прямокутник (усі зразки мають однакову вагу), що додає
      високочастотні артефакти в спектр.

      Вікно Хеммінга "згладжує" краї, роблячи сигнал схожим на дзвін, що знижує
      ці артефакти. Хоча вікно трохи розширює основну частотну складову (main
      lobe), воно значно зменшує бічні піки (side lobes), що робить спектр
      чіткішим. Додається точність аналізу - це важливо для коректного розподілу
      частот на світлодіоди (низькі, середні, високі), щоб шум від різких країв
      не спотворював результат.

      Бібліотека arduinoFFT підтримує й інші вікна:
        1) FFT_WIN_TYP_RECTANGLE (без зважування, прямокутне вікно — найгірше
      для витоків). 2) FFT_WIN_TYP_HANN (вікно Ханна — схоже на Хеммінга, але з
      іншими характеристиками). 3) FFT_WIN_TYP_BLACKMAN (ще сильніше придушення
      бічних піків). Хеммінг — хороший компроміс між роздільністю і придушенням
      витоків.
    */

    FFT.compute(vReal,
                vImag,
                SAMPLES,
                FFT_FORWARD);  // виконує FFT для перетворення сигналу в
                               // частотну область
    /*
      Виконує швидке перетворення Фур’є (FFT), перетворюючи сигнал із часової
      області (зразки з мікрофона) у частотну область (амплітуди частот). Бере
      масиви vReal (реальна частина) і vImag (уявна частина) і обчислює спектр
      частот. vReal — масив реальних значень сигналу (вхід і вихід). vImag —
      масив уявних значень (початково 0, змінюється під час обчислень). SAMPLES
      — розмір масиву, має бути степенем 2. FFT_FORWARD — напрямок перетворення
      (пряме FFT). Інший варіант: FFT_REVERSE (зворотне FFT). Після виконання
      vReal і vImag містять комплексні числа, що представляють частотний спектр.
    */

    FFT.complexToMagnitude(vReal,
                           vImag,
                           SAMPLES);  // 6) Обчислення амплітуд: перехід до величин
                                      // (complexToMagnitude) перетворює комплексні числа у
                                      // величину (амплітуду) для кожної частоти

    ampR = 0;
    ampG = 0;
    ampB = 0;  // 7) Розподіл частот: поділ на баси, середні, високі: R - баси
               // (0–20), G - середні (20–80), B - високі (80–128)
    for (int i = 0; i < SAMPLES; i++) {
      if (i < 20) ampR += abs(vReal[i]);
      else if (i < 80)
        ampG += abs(vReal[i]);
      else
        ampB += abs(vReal[i]);
    }
    ampR /= 20;  // усереднюємо амплітуди басів
    ampG /= 60;  // усереднюємо амплітуди середніх частот
    ampB /= 48;  // усереднюємо амплітуди високих частот

    double totalEnergy = 0;  // 8) Нормалізація: масштабування амплітуд за енергією сигналу.
    for (int i = 0; i < SAMPLES; i++) {
      totalEnergy += vReal[i] * vReal[i];  // енергія = сума квадратів амплітуд
      /*
        енергія = сума квадратів амплітуд бо енергія сигналу як фізична величина
        - пропорційна квадрату амплітуди. Корінь із середньої суми квадратів
        (sqrt(totalEnergy / SAMPLES)) дає середню амплітуду.
      */
    }
    double avgEnergy = sqrt(totalEnergy / SAMPLES);  // нормалізуємо амплітуди за енергією
                                                     // сигналу (відносно середньої енергії)
    /*
      нормалізація - приведення даних до певного масштабу (наприклад, відносно
      середнього значення або енергії). Зменшує вплив фонового шуму, але
      конкретні коефіцієнти залежать від конкретного мікрофона і середовища
      роботи.
    */
    if (avgEnergy > 0) {
      ampR = (ampR / avgEnergy) * 150;  // підсилення басів
      ampG = (ampG / avgEnergy) * 100;  // підсилення середніх частот
      ampB = (ampB / avgEnergy) * 150;  // підсилення високих частот
    }
    /*
      Підсилюємо амплітуди - вирішуємо проблему різної гучності: тихий сигнал не
      "гасить" світлодіоди, а гучний не перевантажує їх Після нормалізації
      амплітуди можуть бути замалими для світлодіодів (0–255). Множники (150,
      100, 150) масштабують їх до видимого діапазону.
    */

    // 9) Ковзне середнє: згладжування значень амплітуд із часом - усереднюємо амплітуди для плавної зміни кольорів
    avgAmpR = (avgAmpR * count + ampR) / (count + 1);
    avgAmpG = (avgAmpG * count + ampG) / (count + 1);
    avgAmpB = (avgAmpB * count + ampB) / (count + 1);
    count++;
    if (count > 50) count = 50;

    //  пороги потрібні для реалізації конкретної ідеї світломузики на led-кружальці - для визначення кількості led які мають світитись
    int porigR = avgAmpR * 0.8;
    int porigG = avgAmpG * 1.2;
    int porigB = avgAmpB * 0.8;

    // для відлагодження виводимо інформацію про амплітуди та середню енергію
    if (millis() - lastPrint >= 5000) {
      Serial.print("Амплітуди: R = ");
      Serial.print(ampR);
      Serial.print(", G = ");
      Serial.print(ampG);
      Serial.print(", B = ");
      Serial.println(ampB);
      Serial.print("Середня енергія: ");
      Serial.println(avgEnergy);
      lastPrint = millis();
    }

    FastLED.clear();  // 10) Керування LED: переведення амплітуд у кольори/яскравість залежно від режиму
    if (mode == 1) {  // перше коло (16 LED)
      if (ampR < porigR) leds_circles[0] = CRGB(255, 0, 0);
      else if (ampR < porigR * 1.25)
        std::fill(leds_circles + 0, leds_circles + 2, CRGB(255, 0, 0));
      else if (ampR < porigR * 1.5)
        std::fill(leds_circles + 0, leds_circles + 3, CRGB(255, 0, 0));
      else if (ampR < porigR * 1.75)
        std::fill(leds_circles + 0, leds_circles + 4, CRGB(255, 0, 0));
      else if (ampR < porigR * 2)
        std::fill(leds_circles + 0, leds_circles + 5, CRGB(255, 0, 0));
      else
        std::fill(leds_circles + 0, leds_circles + 6, CRGB(255, 0, 0));

      if (ampG < porigG) leds_circles[6] = CRGB(0, 255, 0);
      else if (ampG < porigG * 1.3)
        std::fill(leds_circles + 6, leds_circles + 8, CRGB(0, 255, 0));
      else if (ampG < porigG * 1.6)
        std::fill(leds_circles + 6, leds_circles + 9, CRGB(0, 255, 0));
      else if (ampG < porigG * 1.9)
        std::fill(leds_circles + 6, leds_circles + 10, CRGB(0, 255, 0));
      else
        std::fill(leds_circles + 6, leds_circles + 11, CRGB(0, 255, 0));

      if (ampB < porigB) leds_circles[11] = CRGB(0, 0, 255);
      else if (ampB < porigB * 1.3)
        std::fill(leds_circles + 11, leds_circles + 13, CRGB(0, 0, 255));
      else if (ampB < porigB * 1.6)
        std::fill(leds_circles + 11, leds_circles + 14, CRGB(0, 0, 255));
      else if (ampB < porigB * 1.9)
        std::fill(leds_circles + 11, leds_circles + 15, CRGB(0, 0, 255));
      else
        std::fill(leds_circles + 11, leds_circles + 16, CRGB(0, 0, 255));

    } else if (mode == 2) {  // друге коло (12 LED)
      int totalAmp = (ampR + ampG + ampB) / 3;
      int brightness = map(totalAmp, 0, 600, 0, 255);
      std::fill(leds_circles + NUM_LEDS_CIRCLE1, leds_circles + NUM_LEDS_CIRCLES_TOTAL, CRGB(brightness, 0, brightness));

    } else if (mode == 3) {  // обидва кола (28 LED)
      int totalAmp = (ampR + ampG + ampB) / 3;
      int brightness = map(totalAmp, 0, 600, 0, 255);
      std::fill(leds_circles, leds_circles + NUM_LEDS_CIRCLES_TOTAL, CRGB(brightness, 0, brightness));

    } else if (mode == 4) {  // перший квадрат (16 LED)
      if (ampR < porigR) leds_squares[0] = CRGB(255, 0, 0);
      else if (ampR < porigR * 1.25)
        std::fill(leds_squares + 0, leds_squares + 2, CRGB(255, 0, 0));
      else if (ampR < porigR * 1.5)
        std::fill(leds_squares + 0, leds_squares + 3, CRGB(255, 0, 0));
      else if (ampR < porigR * 1.75)
        std::fill(leds_squares + 0, leds_squares + 4, CRGB(255, 0, 0));
      else if (ampR < porigR * 2)
        std::fill(leds_squares + 0, leds_squares + 5, CRGB(255, 0, 0));
      else
        std::fill(leds_squares + 0, leds_squares + 6, CRGB(255, 0, 0));

      if (ampG < porigG) leds_squares[6] = CRGB(0, 255, 0);
      else if (ampG < porigG * 1.3)
        std::fill(leds_squares + 6, leds_squares + 8, CRGB(0, 255, 0));
      else if (ampG < porigG * 1.6)
        std::fill(leds_squares + 6, leds_squares + 9, CRGB(0, 255, 0));
      else if (ampG < porigG * 1.9)
        std::fill(leds_squares + 6, leds_squares + 10, CRGB(0, 255, 0));
      else
        std::fill(leds_squares + 6, leds_squares + 11, CRGB(0, 255, 0));

      if (ampB < porigB) leds_squares[11] = CRGB(0, 0, 255);
      else if (ampB < porigB * 1.3)
        std::fill(leds_squares + 11, leds_squares + 13, CRGB(0, 0, 255));
      else if (ampB < porigB * 1.6)
        std::fill(leds_squares + 11, leds_squares + 14, CRGB(0, 0, 255));
      else if (ampB < porigB * 1.9)
        std::fill(leds_squares + 11, leds_squares + 15, CRGB(0, 0, 255));
      else
        std::fill(leds_squares + 11, leds_squares + 16, CRGB(0, 0, 255));

    } else if (mode == 5) {  // другий квадрат (16 LED)
      int totalAmp = (ampR + ampG + ampB) / 3;
      int brightness = map(totalAmp, 0, 600, 0, 255);
      std::fill(leds_squares + NUM_LEDS_SQUARE1, leds_squares + NUM_LEDS_SQUARES_TOTAL, CRGB(brightness, 0, brightness));

    } else if (mode == 6) {  // обидва квадрати (32 LED)
      int totalAmp = (ampR + ampG + ampB) / 3;
      int brightness = map(totalAmp, 0, 600, 0, 255);
      std::fill(leds_squares, leds_squares + NUM_LEDS_SQUARES_TOTAL, CRGB(brightness, 0, brightness));

    } else if (mode == 7) {  // усе разом (28 + 32 LED)
      int totalAmp = (ampR + ampG + ampB) / 3;
      int brightness = map(totalAmp, 0, 1500, 0, 255);
      std::fill(leds_circles, leds_circles + NUM_LEDS_CIRCLES_TOTAL, CRGB(brightness, 0, brightness));
      std::fill(leds_squares, leds_squares + NUM_LEDS_SQUARES_TOTAL, CRGB(brightness, 0, brightness));
    }

    FastLED.show();
    vTaskDelay(50 / portTICK_PERIOD_MS);
    /*
      Ядро 0 (10 мс): веб-сервер потребує швидкої реакції на запити, тому
      затримка менша. 
      Ядро 1 (50 мс): обробка звуку і світла менш критична до
      часу, а більша затримка економить ресурси. Вибір числа залежить від
      частоти оновлення (50 мс = 20 Гц, достатньо для плавності світломузики).
    */
  }
}

void setup() {
  Serial.begin(115200);
  pinMode(34, INPUT);
  FastLED.addLeds<WS2812B, LED_PIN_CIRCLES, GRB>(leds_circles, NUM_LEDS_CIRCLES_TOTAL);
  FastLED.addLeds<WS2812B, LED_PIN_SQUARES, GRB>(leds_squares, NUM_LEDS_SQUARES_TOTAL);
  FastLED.setBrightness(100);

  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("");
  Serial.println("Wi-Fi підключено!");
  Serial.print("IP-адреса: ");
  Serial.println(WiFi.localIP());

  xTaskCreatePinnedToCore(webServerTask, "WebServerTask", 8192, NULL, 1, NULL, 0);
  xTaskCreatePinnedToCore(lightMusicTask, "LightMusicTask", 8192, NULL, 1, NULL, 1);
  /*
    Параметри:
      lightMusicTask — функція-завдання.
      "LightMusicTask" — ім’я для дебагу.
      8192 — розмір стека в байтах.
      NULL — параметри для функції (не використовуються).
      1 — пріоритет (0 — найнижчий, до 24 на ESP32).
      NULL — вказівник на handle задачі (не потрібен).
      1 — ядро (0 або 1).

    У ESP32 і FreeRTOS можна призначати кілька завдань на одне ядро. FreeRTOS
    розподіляє час між задачами на одному ядрі за пріоритетами. Вищий пріоритет
    (наприклад, 2) перериває нижчий (1). Якщо однаковий, час ділиться порівну.
    Розмір стеку залежить від пам’яті ESP32 (зазвичай до 320 КБ SRAM). 8192 байт
    — типове значення. Вибір: Занадто малий стек призведе до збою (stack
    overflow). Тестується залежно від складності задачі.
  */
}

void loop() {}  // порожня функція, оскільки вся логіка реалізована в задачах FreeRTOS