Können wir hiermit ein LKS32MC target dem Keil projekt in HoverBoardGigaDevice zufügen:

´´´
I've been investigating support for the LKS32MCR11C6T8M used on newer hoverboard controllers.

I was able to locate the official LinkoSemi SDK from the manufacturer's website "https://www.lksmcu.com/index.php/LKS05Series/" , including:

Linko LKS05x CMSIS Device Pack (v1.1.5)
Peripheral driver library
Demo projects
Evaluation board package
Flash programming algorithm (.FLM)
Startup files and CMSIS headers
English datasheet and user manual

I installed the official Device Pack into Keil MDK successfully, and the official GPIO demo project builds successfully using Arm Compiler 6 (with C99 enabled). This confirms the SDK and toolchain are usable.

My hardware (LKS32MCR11C6T8M hoverboard controller) is still in transit, so I haven't been able to verify flashing or firmware execution yet.

Once the hardware arrives, I'll test flashing with ST-Link, compare the LKS32MCR11C6T8M against the LKS32MC051C6T8 SDK, and report back with results. If they're compatible, I'll also be happy if any of you who have the board can test the demo projects and confirm, so that we can extend the hoverboard firmware to LKS32MCR11C6T8M.
´´´


Auf Github berichten Nutzer zuletzt öfter von dem Problem, dass keine der sechs Hall Input permutationen den bldc motor ruckelfrei ansteuern lassen. Erst das Vertauschen zwei der drei bldc Motorkabel funktioniert. Ich glaube das ist mir auch schonmal passiert mit den boards die in 10" Hoverboards verbaut sind. Da ist der Color-Code der drei BLDC kabel nicht identisch mit den gelb,grün,blau der hall inputs. Was ich aber nicht verstehe: warum kann eine Vertauschung der hall inputs in einer der defines wie defines_2-1-1.h eine Vertauschung der bldc-kabel nicht wieder ausgleichen:

	// Hall sensor defines
	#define HALL_A	PB11
	#define HALL_B	PF1
	#define HALL_C	PC14

Die 6 mosfet out-pins sind ja durch den TIMER0 fast bei allen boards festgelegt auf

	// Brushless Control DC (BLDC) defines
	#define BLDC_GH PA10		// green	, Tommyboi2001 all bldc pins same as 2.0
	#define BLDC_GL PB15		
	#define BLDC_BH PA9			// blue
	#define BLDC_BL PB14		
	#define BLDC_YH PA8			// yellow
	#define BLDC_YL PB13		


Eine neue Option HALL_INVERT_ALL scheint mir die richtige Lösung.
Die damit verbunden Invertierung der drei Hall-Bits müssen wir dann nur in eine kleine helper-function in bldc.c einbauen,
die dann auch `hall = digitalRead(HALL_A) + digitalRead(HALL_B)*2 + digitalRead(HALL_C)*4;		` in bldcSINE.cd ersetzt ?

Wir könnten als default in defines.h schreiben, wenn nicht REMOTE_AUTODETECT gewählt ist:

	#ifndef HALL_INVERT_ALL
		#define HALL_INVERT_ALL false
	#endif

Und für Autodetect in RemoteAutodetect.c eine echte Variable dort zufügen:

	uint32_t HALL_A = TODO_PIN;
	uint32_t HALL_B = TODO_PIN;
	uint32_t HALL_C = TODO_PIN;
	boolean HALL_INVERT_ALL = false;

Stimmst Du dem zu ? Siehst Du noch Probleme ?


Danke für den Ausblick auf die Implementierung in RemoteAutodetect.c

Aber lasse uns jetzt erstmal HALL_INVERT_ALL mit 1 oder 0 in defines.h, bldc.h , bldc.c und bldcSINE.c (und weitere?) implementieren.

Dann könnten User HALL_INVERT_ALL für REMOTE_AUTODETECT schon manuell in defines.c auf 1 setzen, und die fehlenden 6 Permutationen testen.


Okay, ich habe HALL_INVERT_ALL als Variable auch in remoteAutodetect.c Zeile 344 eingebaut, und auch in defines.h. Dort habe ich auch die #ifndef in den #else Zweig von `#if defined(REMOTE_AUTODETECT)` verschoben.
Der code compiliert erfolgreich. Haben wir damit die Voraussetzung für die Implementierung in remoteAutodetect geschaffen ?


HALL_INVERT_ALL sollte in keiner config.h verwendet werden, da die Angabe zu den layout defines gehört.

Ich habe in Deinen Antworten gesehen, dass nicht alle code dateien utf8 Format haben. Das sollten wir ändern ?


Danke. 
Jetzt fasse mir nochmal zusammen, wie Du HALL_INVERT_ALL ab Zeile 1329 in die Schleife von AUTODETECT_Stage_HallOrder implementieren willst.


Okay, ich hab den Code jetzt mit meinem 2.1.11 test-setup getestet.

Dabei musste ich den reset von iRepeat mittels `if (posOld == posAuto) iRepeat = 0; `einschränken, weil er sonst wohl manchmal fälschlicher Weise den Counter auf null setzt.

Erstmal verifizierte ich, dass AUTODETECT_Stage_HallOrder mit der der normalen Verkabelung des bldc Motors funktioniert.
Und danach auch gleich noch AUTODETECT_Stage_CurrentDC um zu prüfen dass nun auch wirklich der Motor ruckelfrei angesteuert wird.

Dann hab ich zwei bldc Kabel vertauscht und in der Tat wurde nun eine HALL_INVERT_ALL=1 Permutation gefunden.
Allerdings funktioniert AUTODETECT_Stage_CurrentDC nicht.
Aber eigentlich müsste HALL_INVERT_ALL=1 doch noch von der vorigen AUTODETECT_Stage_HallOrder aktiv sein ?


Nein Dein Code funktioniert nicht. Jetzt passiert der reset von iRepeat sogar manchmal nicht beim Wechsel von 6 auf 1. Ich beobachte mit StmStudio. Nun hinkt in StmStudio posOld immer um 1 hinter posAuto hinterher. posOld ist immer identisch mit posNew2
Mit meinem Code hingegen lagen sie genau übereinander, weswegen nach 20 iRepeat das auch keine Zufall sein konnte.

Schau mal ob ich Deinen code richtig umgesetzt habe.

Aber ich sehe nun, dass mit meinem Code tatsächlich bei vertauschten Kabeln die falsche Permutation gefunden wurde.
Damals genau jene, bei der posAuto um 1 posOld (und posNew2) hinterher hinkt.

Wir brauchen eine bessere Logik für korrekte Hall Order ?


Nein das ist mir zu kompliziert. Ich bin also wieder zu meinem ursprnglichen Code zurück. 
Nur posAutoApplied habe ich beibehalten.

Jetzt werden die unvertauschten bldc-kabel wieder zuverlässig erkannt und der currentDC Test läuft auch sauber.
Mit vertauschtem Paar wird auch ein Konfiguration erkannt, und da zeigt mir StmStudio wie posAutoApplied und posOld und posNew2 sauber übereinander liegen. Bis auf seltene Stellen wenn posNew2 um 1 voreilt, was aber von meiner Logik ignoriert wird.
Trotzdem läuft der currentDC test nicht.
Mir fällt aber auch auf, das die iRepeat deutlich schneller auf 21 hochgezählt werden.
Was aber eigentlich korrekt ist, denn bei jedem neuen posNew passt die HallOrder und iRepeat wird um 1 hochgezählt.
Nur beim Wechsel von 6 auf 1 zählt meine Logik nicht hoch. Damit kann ich aber leben.
Denn bei nicht vertauschtem Kabeln, zählt iRepeat noch langsamer hoch, erkennt aber immer noch zuverlässig.

Das Problem scheint mir also nicht in der Detektion der Permutation zu liegen, sondern in anschließendem normalen Motor-Modus.


Ohne Vertauschung läuft der Motor sauber mit

#define HALL_A		PB11
#define HALL_B		PC14
#define HALL_C		PF1


Wenn ich gelb und grün vertausche, also A und C, dann wird das detektiert:

#define HALL_A		PC14
#define HALL_B		PB11
#define HALL_C		PF1

Das sieht jetzt falsch aus, aber ich hab mal alle 12 Permutationen durchführen lassen. Und es ist genau nur diese Permutation die als posXY synchron hat.


Ja das funktioniert, der currentDC Test funktioniert sauber !

Das wurde nun detektiert:

#define HALL_A		PB11
#define HALL_B		PF1
#define HALL_C		PC14


Okay, jetzt der flash write/read. 
oConfig.wState scheint noch gar nicht benutzt zu werden, da können wir also ein Bit für HALL_INVERT_ALL verwenden.


Okay, ich hab noch AUTODETECT_Stage_Results aktualisiert mit:

			if (HALL_INVERT_ALL)	sprintf(sMessage + strlen(sMessage), "#define HALL_INVERT_ALL 1\n");

Damit ist die firmware bereit um auf Github hochgeladen zu werden ?


Ich hab schon ausschalten und anschalten getestet, das klappt.

Und die vielen Änderungen die mir Github-Desktop anzeigt sind nur Deine vielen utf8 Anpassungen.

Wir könnten höchstens noch bei der neuen Logik von HALL_INVERT_ALL aufräumen, also ein paar der globalen StmStudio Variablen entfernen:

uint8_t posNew2 = 0;
uint8_t posAutoApplied = 0;
uint8_t posReference = 0;

uint8_t AutodetectBldc(uint8_t posNew,uint16_t buzzerTimer)

