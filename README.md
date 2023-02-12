# Datorarkitektur (del VI) - Avbrottsimplementering
Sjätte och sista delen i en serie som behandlar CPU-konstruktion i mjukvara. 
En simulerad processor som baseras på mikrodator ATmega328P implementeras i terminalmiljö,
där klockpulser, in- och utdata implementeras via inmatning samt utskrift från terminalen.

Denna del behandlar avbrottsimplementering för PCI-avbrott, vilket möjliggör avbrott vid
logisk förändring av insignaler. För detta ändamål implementeras monitorering av insignaler
på samtliga I/O-portar (via innehållet i pin-register PINB - PIND).
Monitorering sker under varje klockpuls, medan kontroll efter IRQ (interrupt requests) 
enbart genomförs i slutet av varje execute-fas.

Även instruktioner SEI, CLI samt RETI implementeras för att kunna aktivera 
respektive inaktivera avbrott samt genomföra återhopp efter slutfört avbrott.

För en given insignal gäller att ifall motsvarande maskbit har ettställs i tillhörande 
maskregister PCMSK0 - PCMSK2 jämförs nuvarande insignal mot föregående insignal. 
Om dessa är olika ettställs motsvarande interrupt-flagga PCIF0 - PCIF2 i flaggregistret PCIFR.

Vid kontroll efter eventuella IRQ kontrolleras först och främst så att I-flaggan är ettställd
(den globala interrupt-flaggan i statusregistret). Om detta är fallet kontrolleras interrupt-flaggorna 
PCIF0 - PCIF2 i flaggregistret PCIFR. Om en given interrupt-flagga är ettställd samtidigt som 
motsvarande enable-bit PCIE0 - PCIE2 i kontrollregistret PCICR är ettställd genereras ett avbrott. 
Enbart ett avbrott genereras åt gången, så är fler interrupt-flaggor samt motsvarande enable-bitar 
ettställda kommer dessa att behandlas när aktuellt avbrott är slutfört.

Innan ett avbrott genereras nollställs interrupt-flaggan PCIF0 - PCIF2 i fråga samt I-flaggan. 
Returadressen efter avbrottet (programräknarens innehåll) samt innehållet i statusregistret 
(innan I-flaggan nollställs) sparas på stacken. Programhopp sker sedan till motsvarande avbrottsvektor. 
När avbrottet avslutas hämtas returadressen samt statusregistrets innehåll från stacken.

Avbrottsimplementeringen testas via ett program, där en lysdiod togglas vid nedtryckning av en tryckknapp.
PCI-avbrott har aktiverats på tryckknappen i fråga så att programhopp sker till motsvarande avbrottsrutin 
PCINT0_vect vid nedtryckning.

Filen "embedded_computer_system_part6_start_code.zip" innehåller startkoden med färdig programkod för 
test av avbrottsimplementeringen. Jämfört med föregående del så genomför ALU:n subtraktion via addition med
2-komplement (så som sker i hårdvaran) i stället för en regelrätt addition.

Samtliga .c- och .h-filer utgörs av processorn med färdig interrupt-implementering.

Se video tutorial här:
https://youtu.be/SXwybMwmal8