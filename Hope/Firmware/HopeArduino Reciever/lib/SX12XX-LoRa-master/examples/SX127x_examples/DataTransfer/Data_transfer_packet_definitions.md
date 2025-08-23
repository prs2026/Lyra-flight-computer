## Data Transfer packet Definitions

    0xA0 	Request to TX, DTSegmentWrite, Header Length 6, Data length Max, 245
    0xA1 	ACK to RX, DTSegmentWriteACK, Header Length 6, Data length Max, 245
    0xA2 	NACK to RX, DTSegmentWriteNACK, Header Length 6, Data length Max, 245
    
    0xA4 	Request to TX, DTFileOpen, Header Length 12, Data length Max, 239
    0xA5 	ACK to RX, DTFileOpenACK, Header Length 12, Data length Max, 239
    0xA6 	NACK to RX, DTFileOpenNACK, Header Length 12, Data length Max, 239
    
    0xA8 	Request to TX, DTFileClose, Header Length 12, Data length Max, 239
    0xA9 	ACK to RX, DTFileCloseACK, Header Length 12, Data length Max, 239
    0xAA 	NACK to RX, DTFileCloseNACK, Header Length 12, Data length Max, 239
    
    0xAC 	Request to TX, DTFileSeek, Header Length 9, Data length Max, 242
    0xAD 	ACK to RX, DTFileSeekACK, Header Length 9, Data length Max, 242
    0xAE 	NACK to RX, DTFileSeekNACK, Header Length 9, Data length Max, 242
    
    0xB0 	Request to TX, DTStart, Header Length 6, Data length Max, 245
    0xB1 	ACK to RX, DTStartACK, Header Length 6, Data length Max, 245
    0xB2 	NACK to RX, DTStartNACK, Header Length 6, Data length Max, 245
    
    0xB4 	Request to TX, DTWake, Header Length 6, Data length Max, 245
    0xB5 	ACK to RX, DTWakeACK, Header Length 6, Data length Max, 245
    0xB6 	NACK to RX, DTWakeNACK, Header Length 6, Data length Max, 245
    
    
    0xA0 	
    DTSegmentWrite, Header Length 6, Data length Max, 245
    Header
    Byte	Purpose
    0	0xA0
    1	Flags
    2	Header length
    3	Data  length
    4	SegmentNum0
    5	SegmentNum1
    Data 	
    Byte	Purpose
    6	DataArray  Start
    7	More data etc
    
    
    0xA1 	
    DTSegmentWriteACK, Header Length 6
    Header
    Byte	Purpose
    0	0xA1
    1	Flags
    2	Header length
    3	Data  length
    4	SegmentNum0
    5	SegmentNum1
    
    0xA2 	
    DTSegmentWriteACK, Header Length 6
    Header
    Byte	Purpose
    0	0xA2
    1	Flags
    2	Header length
    3	Data  length
    4	Required SegmentNum0
    5	Required SegmentNum1
    
    
    0xA4 	
    DTFileOpen, Header Length 12, Data length Max, 239
    Header
    Byte	Purpose
    0	0xA4
    1	Flags
    2	Header length
    3	Data  length
    4	Filelength0
    5	Filelength1
    6	Filelength2
    7	Filelength3	
    8	FileCRC0
    9	FileCRC1
    10  SegmentSize
    11   Unused 
    Data 	
    Byte	Purpose
    12 	FilenameArray  Start
    13	More FilenameArray etc
    
    
    0xA5 	
    DTFileOpenACK, Header Length 12
    Header
    Byte	Purpose
    0	0xA5
    1	Flags
    2	Header length
    3	Data  length
    4	Filelength0
    5	Filelength1
    6	Filelength2
    7	Filelength3	
    8	FileCRC0
    9	FileCRC1
    10  SegmentSize
    11   Unused 
    
    0xA6 	
    DTFileOpenNACK, Header Length 12
    Header
    Byte	Purpose
    0	0xA6
    1	Flags
    2	Header length
    3	Data  length
    4	Filelength0
    5	Filelength1
    6	Filelength2
    7	Filelength3	
    8	FileCRC0
    9	FileCRC1
    10  SegmentSize
    11   Unused 
    
    
    0xA8 	
    DTFileClose, Header Length 12, Data length Max, 239
    Header
    Byte	Purpose
    0	0xA8
    1	Flags
    2	Header length
    3	Data  length
    4	Filelength0
    5	Filelength1
    6	Filelength2
    7	Filelength3	
    8	FileCRC0
    9	FileCRC1
    10  SegmentSize
    11   Unused 
    Data 	
    Byte	Purpose
    12 	FilenameArray  Start
    13	More FilenameArray etc
    
    
    0xA9 	
    DTFileCloseACK, Header Length 12
    Header
    Byte	Purpose
    0	0xA9
    1	Flags
    2	Header length
    3	Data  length
    4	Filelength0
    5	Filelength1
    6	Filelength2
    7	Filelength3	
    8	FileCRC0
    9	FileCRC1
    10  SegmentSize
    11   Unused 
    
    0xAA 	
    DTFileCloseNACK, Header Length 12
    Header
    Byte	Purpose
    0	0xAA
    1	Flags
    2	Header length
    3	Data  length
    4	Filelength0
    5	Filelength1
    6	Filelength2
    7	Filelength3	
    8	FileCRC0
    9	FileCRC1
    10  SegmentSize
    11   Unused 
    
    0xAC 	
    DTDataSeek, Header Length 9, Data length Max, 242
    Header
    Byte	Purpose
    0	0xAC
    1	Flags
    2	Header length
    3	Data  length
    4	DataSeek0
    5	DataSeek1
    6	DataSeek2
    7	DataSeek3	
    8Unused 
    Data 	
    Byte	Purpose
    9 	FilenameArray  Start
    13	More FilenameArray etc
    
    
    0xAD 	
    DTDataSeekACK, Header Length 9, Data length Max, 242
    Header
    Byte	Purpose
    0	0xAD
    1	Flags
    2	Header length
    3	Data  length
    4	DataSeek0
    5	DataSeek1
    6	DataSeek2
    7	DataSeek3	
    8Unused 
    
    
    0xAE 	
    DTDataSeekNACK, Header Length 9, Data length Max, 242
    Header
    Byte	Purpose
    0	0xAE
    1	Flags
    2	Header length
    3	Data  length
    4	DataSeek0
    5	DataSeek1
    6	DataSeek2
    7	DataSeek3	
    8Unused 
    
    
    0xB0 	
    DTStart, Header Length 6, Data length Max, 245
    Header
    Byte	Purpose
    0	0xB0
    1	Flags
    2	Header length
    3	Data  length
    4	Unused 
    5	Unused 
    
    Data 	
    Byte	Purpose
    6 SegmentSize
    7 LastSegmentSize
    8 TXtimeoutmS0
    9 TXtimeoutmS1
    10 TXtimeoutmS2
    11 TXtimeoutmS3
    12 RXtimeoutmS0
    13 RXtimeoutmS1
    14 RXtimeoutmS2
    15 RXtimeoutmS3
    16 ACKtimeoutDTmS0
    17 ACKtimeoutDTmS1
    18 ACKtimeoutDTmS2
    19 ACKtimeoutDTmS3
    20 ACKdelaymS0
    21 ACKdelaymS1
    22 ACKdelaymS2
    23 ACKdelaymS3
    24 packetdelaymS0
    25 packetdelaymS1
    26 packetdelaymS2
    27 packetdelaymS3
    28 Frequency0
    29 Frequency1
    30 Frequency2
    31 Frequency3
    32 Offset0
    33 Offset1
    34 Offset2
    35 Offset3
    36 Spreading Factor
    37 Bandwidth
    38 Coding Rate
    39 Optimisation
    40 TXPower
    41 Implicit/Explicit
    42 TXattempts0
    43 TXattempts1
    44 HeaderSizeMax
    45 DataSizeMax
    
    
    0xB1 	
    DTStartACK, Header Length 6, Data length Max, 245
    Header
    Byte	Purpose
    0	0xB1
    1	Flags
    2	Header length
    3	Data  length
    4	Unused 
    5	Unused 
    
    
    0xB2 	
    DTStartNACK, Header Length 6, Data length Max, 245
    Header
    Byte	Purpose
    0	0xB2
    1	Flags
    2	Header length
    3	Data  length
    4	Unused 
    5	Unused 
    
    0xB4 	
    DTWake, Header Length 6, Data length Max, 245
    Header
    Byte	Purpose
    0	0xB4
    1	Flags
    2	Header length
    3	Data  length
    4	Unused 
    5	Unused 
    
    
    0xB5 	
    DTWakeACK, Header Length 6, Data length Max, 245
    Header
    Byte	Purpose
    0	0xB5
    1	Flags
    2	Header length
    3	Data  length
    4	Unused 
    5	Unused 
    
    
    0xB6 	
    DTWakeNACK, Header Length 6, Data length Max, 245
    Header
    Byte	Purpose
    0	0xB6
    1	Flags
    2	Header length
    3	Data  length
    4	Unused 
    5	Unused 