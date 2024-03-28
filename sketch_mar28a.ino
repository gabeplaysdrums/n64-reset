#define VERBOSE 0

#define PIN_READ( pin )  (PINB & (1 << pin))
#define MICROSECOND_NOPS "nop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\n"

#define N64_PIN        6

// connect to reset pin of n64
// NEVER make N64_RESET_PIN HIGH, it can damage your console
#define N64_RESET_PIN        4

#define N64_PREFIX     9
#define N64_BITCOUNT  32

//if you want to use different buttons, see http://www.qwertymodo.com/hardware-projects/n64/n64-controller
#define BUTTON_A            (1 << 0)
#define BUTTON_B            (1 << 1)
#define BUTTON_Z            (1 << 2)
#define BUTTON_START        (1 << 3)
#define BUTTON_DPAD_UP      (1 << 4)
#define BUTTON_DPAD_DOWN    (1 << 5)
#define BUTTON_DPAD_LEFT    (1 << 6)
#define BUTTON_DPAD_RIGHT   (1 << 7)
#define BUTTON_L            (1 << 10) 
#define BUTTON_R            (1 << 11)
#define BUTTON_C_UP         (1 << 12)
#define BUTTON_C_DOWN       (1 << 13)
#define BUTTON_C_LEFT       (1 << 14)
#define BUTTON_C_RIGHT      (1 << 15)

static const char* const buttonNames[] {
    " A",
    " B",
    " Z",
    " START",
    " DPAD_UP",
    " DPAD_DOWN",
    " DPAD_LEFT",
    " DPAD_RIGHT",
    nullptr,
    nullptr,
    " L",
    " R",
    " C_UP",
    " C_DOWN",
    " C_LEFT",
    " C_RIGHT"
};

// Reset the console if any of the following button combos are pressed
constexpr uint16_t RESET_BUTTON_COMBOS[] {
    (BUTTON_L | BUTTON_R | BUTTON_C_UP | BUTTON_C_RIGHT | BUTTON_DPAD_DOWN),
    (BUTTON_L | BUTTON_R | BUTTON_C_UP | BUTTON_C_RIGHT | BUTTON_DPAD_DOWN | BUTTON_DPAD_LEFT),
    (BUTTON_L | BUTTON_R | BUTTON_C_UP | BUTTON_C_RIGHT | BUTTON_DPAD_DOWN | BUTTON_DPAD_RIGHT),
};

void setup()
{
    PORTB = 0x00;
    DDRB  = 0x00;
    PORTD = 0x00;
    DDRD  = 0x00;

    pinMode(N64_RESET_PIN, INPUT);
    // NEVER make N64_RESET_PIN HIGH, it can damage your console
    digitalWrite(N64_RESET_PIN, LOW);

    Serial.begin( 115200 );
}

#define WAIT_FALLING_EDGE( pin ) while( !PIN_READ(pin) ); while( PIN_READ(pin) );

// Declare some space to store the bits we read from a controller.
unsigned char rawData[ 128 ];

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Performs a read cycle from one of Nintendo's one-wire interface based controllers.
// This includes the N64 and the Gamecube.
//     pin  = Pin index on Port C where the data wire is attached.
//     bits = Number of bits to read from the line.
template< unsigned char pin >
void read_oneWire( unsigned char bits )
{
    unsigned char *rawDataPtr = rawData;

read_loop:

    // Wait for the line to go high then low.
    WAIT_FALLING_EDGE( pin );

    // Wait ~2us between line reads
    asm volatile( MICROSECOND_NOPS MICROSECOND_NOPS );

    // Read a bit from the line and store as a byte in "rawData"
    *rawDataPtr = PIN_READ(pin);
    ++rawDataPtr;
    if( --bits == 0 ) return;

    goto read_loop;
}

// Verifies that the 9 bits prefixing N64 controller data in 'rawData'
// are actually indicative of a controller state signal.
inline bool checkPrefixN64 ()
{
    if( rawData[0] != 0 ) return false; // 0
    if( rawData[1] != 0 ) return false; // 0
    if( rawData[2] != 0 ) return false; // 0
    if( rawData[3] != 0 ) return false; // 0
    if( rawData[4] != 0 ) return false; // 0
    if( rawData[5] != 0 ) return false; // 0
    if( rawData[6] != 0 ) return false; // 0
    if( rawData[7] == 0 ) return false; // 1
    if( rawData[8] == 0 ) return false; // 1
    return true;
}

void printN64Data( unsigned char first, unsigned char count )
{
    for(unsigned char i = first ; i < first + count ; i++ ) {
        if ((i - first) % 4 == 0) Serial.write(' ');
        Serial.write( rawData[i] ? '1' : '0' );
    }

    Serial.write(' ');

    for (unsigned char i = 0; i < 16; i++) {
        if (rawData[first + i] && buttonNames[i]) {
            Serial.write(buttonNames[i]);
        }
    }

    Serial.write('\n');
}

uint16_t N64_getPressed(unsigned char first, unsigned char count) {
    uint16_t pressed = 0;
    for( unsigned char i = 0 ; i < count ; i++ ) {
        if (rawData[first + i]) {
            pressed |= (1 << i);
        }
    }
    return pressed;
}

void resetN64() {
    // make reset pin floating (if it is not already floating)
    pinMode(N64_RESET_PIN, INPUT);
    // make output of reset pin low
    // after this line the pin remains floating, but we are making it low already so that we do not
    // accidentilly send 5V to reset pin when we stop make it floating
    // NEVER make N64_RESET_PIN HIGH, it can damage your console
    digitalWrite(N64_RESET_PIN, LOW);
    // make reset pin output, so it becomes low
    pinMode(N64_RESET_PIN, OUTPUT);
    // wait 100 ms
    delay(100);
    // make reset pin floating
    pinMode(N64_RESET_PIN, INPUT);
    
    // cannot reset again for 5 seconds
    delay(5000);
}

enum class ComboDetectionState : uint8_t {
    WaitForClear = 0,
    WaitForCombo = 1,

    Init = WaitForClear
};

static ComboDetectionState detectionState = ComboDetectionState::Init;

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Arduino sketch main loop definition.
void loop()
{
    noInterrupts();
    read_oneWire< N64_PIN >( N64_PREFIX + N64_BITCOUNT );
    interrupts();
    if( checkPrefixN64() ) {
#if VERBOSE
        printN64Data(N64_PREFIX, N64_BITCOUNT);
#endif

        const auto pressed = N64_getPressed(N64_PREFIX, 16);

        switch (detectionState) {
            case ComboDetectionState::WaitForClear: {
                if (pressed == 0) {
#if !VERBOSE
                    printN64Data(N64_PREFIX, N64_BITCOUNT);
#endif
                    Serial.println("Clear detected!");
                    detectionState = ComboDetectionState::WaitForCombo;
                }
                break;
            }
            case ComboDetectionState::WaitForCombo: {
                for (const auto combo : RESET_BUTTON_COMBOS) {
                    if (pressed == combo) {
#if !VERBOSE
                        printN64Data(N64_PREFIX, N64_BITCOUNT);
#endif
                        Serial.println("Reset button combo detected!");
                        resetN64();
                        detectionState = ComboDetectionState::Init;
                        break; // for-loop
                    }
                }
                break; // switch
            }
        }

        
    }
}