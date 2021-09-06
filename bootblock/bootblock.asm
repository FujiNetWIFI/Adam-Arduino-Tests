; boot loader
; EOSDEF -- Library of essential routines for EOS

EosStart        EQU     0fc30h  ;EOSStart: reset EOS
ConsDisp        EQU     0fc33h  ;ConsoleDisplay: A=character to display (raw output)
ConsInit        EQU     0fc36h  ;ConsoleInitialize: D=top E=left B=width C=height HL=start
ConsOut         EQU     0fc39h  ;ConsOut: A=character to display, with escape characters
ReadKeyboard    EQU     0fc6ch  ;ReadKeyboard: Returns A=key
GotoWP          EQU     0fce7h  ;GotoWP: Starts SmartWRITER
PutAscii        EQU     0fd17h  ;PutASCII: Fills out characters 0x20 to 0x7e
WriteReg        EQU     0fd20h  ;WriteVDPRegister: B=reg, C=value
FillVRam        EQU     0fd26h  ;FillVRam: HL=start, A=character, DE=length
WriteVRam       EQU     0fd1ah  ;WriteVRam: HL=ram address, BC=length, DE=vram address
ReadVRam        EQU     0fd1dh  ;ReadVRam: HL=ram address, BC=length, DE=vram address
InitTable       EQU     0fd29h  ;InitializeVDPTable: A=table #, HL=location
LoadAscii       EQU     0fd38h  ;LoadASCII: HL=first character, BC=count of characters, DE=vram destination
Read1Block      EQU     0fcf3h  ;Actually ReadBlock Input: A=device, HL=memory, BCDE=block
Write1Block     EQU     0fcf6h  ;Actually WriteBlock Output: A=device, HL=memory, BCDE=block
CurrentDev      EQU     0fd6fh  ;Current device ID
DiskA           EQU     04h
DiskB           EQU     05h
TapeA           EQU     08h
TapeB           EQU     18h

DefPatternTable EQU     0h
DefNameTable    EQU     1800h
DefSprAttrTable EQU     1b00h
DefColorTable   EQU     2000h
DefSprPatTable  EQU     3800h

        ORG     $C800

Boot:
        LD      A,B
        LD      (CurrentDev),A

        CALL    Text

        LD      HL,MsgBlock0
        LD      DE,32*4+16-(14/2)+DefNameTable
        LD      BC,14
        CALL    WriteVRam

loop:
	JP	loop
	
Text:
InitVDP:
        LD      BC,0000h        ;Graphic mode 1
        CALL    WriteReg
        LD      BC,01e0h        ;Graphic mode 1
        CALL    WriteReg
        LD      BC,705h ;Clear background color
        CALL    WriteReg
        LD      A,0     ;Table of sprite attributes
        LD      HL,1b00h
        CALL    InitTable
        LD      A,1     ;Table of sprite patterns
        LD      HL,3800h
        CALL    InitTable
        LD      A,2     ;Name table
        LD      HL,1800h
        CALL    InitTable
        LD      A,3     ;Pattern table
        LD      HL,0
        CALL    InitTable
        LD      A,4     ;Color table
        LD      HL,2000h
        CALL    InitTable
        CALL    LoadAscii
        LD      HL,2000h        ;Color table
        LD      A,0f0h  ;White on black -- or green
        LD      DE,10h  ;first half of table
        CALL    FillVRam
        LD      HL,2010h        ;Color table
        LD      A,07fh  ;Black on White -- or red
        LD      DE,10h  ;first half of table
        CALL    FillVRam
        LD      BC,1f17h        ;31x23
        LD      DE,0            ;Upper left corner
        LD      HL,3800h        ;Name table
        CALL    ConsInit
        RET

MsgBlock0:
        DB      "BLOCK 0 Loaded"

