TToolParams tp = {
    "ABEL",
    0x3110,         // BegMark
    TOOLIDENT,         // Ident;
    1,                  // Serial;
    VERSION,            // Version;
    NUMBLOCKS,           // NumBlocks;
    1,                  // NumPages;
    SIZEPAGE,           // SizePage;
    ADDSIZEPAGE,

    64,                 // FreeBlock;
    0,                  // DefectBlocks;
    SIZEOFDATA,         // SizeOfData;   // in bytes
    10,		      // SizeOfService;
    115,                // Length;               // in cm
    110,                // Diameter;             // in mm
    32,                 // Weight;              // in kg
    stDownhole,         // Status;               // Uphole or downhole, 

    8000000,            // MainFreq;                 // Frequency in KHz
    0,                  // LeftTicks;
    0,                  // WorkSecCounter;
    0x7777,              // uRes;
    SIZEPAGE/SIZEOFDATA,    //ItemInPage;
    0, 			// PageInItem;
    9600, 768, 0, 0, 0,        // Reserv[5];         // 16
    0x1963,             // EEEndMark;	// 64

    12,                  // NumChannel;         // 2
    12,                  //RealChannel;        // 2
    1, 2, 3, 4, 5, 6,   // Reserv1[6];         // 14
    {
        { "DTime", tsSave, todRtc, sizeof( TRtcTime ), 33, 1, sizeof( TRtcTime ), 0, 0, 1, 1, 1.0, 0.0, 37 },
        { "Mark", tsSave+tsHex, todI2, 2, 33, 1, 2, 0, 0, 1, 1, 1.0, 0.0, 37 },
        { "Volt", tsSave, todR4, 4, 33, 1, 4, 0, 0, 1, 1, 1.0, 0.0, 37 },
        { "Temp", tsSave, todR4, 4, 33, 1, 4, 0, 0, 1, 1, 1.0, 0.0, 37 },
        { "XYZ", tsSave, todR4, 4, 33, 3, 12, 0, 0, 1, 1, 1.0, 0.0, 37 },
        { "I1", tsSave, todR4, 4, 33, 4, 16, 0, 0, 1, 1, 1.0, 0.0, 37 },
        { "U1", tsSave, todR4, 4, 33, 4, 16, 0, 0, 1, 1, 1.0, 0.0, 37 },
        { "I2", tsSave, todR4, 4, 33, 4, 16, 0, 0, 1, 1, 1.0, 0.0, 37 },
        { "U2", tsSave, todR4, 4, 33, 4, 16, 0, 0, 1, 1, 1.0, 0.0, 37 },
        { "B1", tsSave, todR4, 4, 33, 1, 4, 0, 0, 1, 1, 1.0, 0.0, 37 },
        { "B2", tsSave, todR4, 4, 33, 1, 4, 0, 0, 1, 1, 1.0, 0.0, 37 },
        { "B3", tsSave, todR4, 4, 33, 1, 4, 0, 0, 1, 1, 1.0, 0.0, 37 }
    },

};

