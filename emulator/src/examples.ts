/** Example programs from the original articles (Jugend+Technik 12/1987, H. Hoyer). */
export interface Example {
  name: string
  note: string
  listing: string
}

export const EXAMPLES: Example[] = [
  {
    name: 'Hallo',
    note: 'Smallest test: prints a greeting.',
    listing: `10 PRINT "HALLO TINY"
20 END`,
  },
  {
    name: 'Zählprogramm',
    note: 'Counts from an input value up to 50.',
    listing: `5 PRINT
10 INPUT "A:"A
20 LET A=A+1
30 PRINT "A="A
40 IF A<50 THEN GOTO 20
100 END`,
  },
  {
    name: 'Mittelwert',
    note: 'Running sum and mean of the numbers you enter (OFF = SHIFT+ENTER stops).',
    listing: `10 LET I=0
20 LET S=0
30 INPUT "WERT:"A
40 LET I=I+1
50 LET S=S+A
60 PRINT "SUMME="S
70 PRINT "MITTEL="S/I
80 GOTO 30`,
  },
  {
    name: 'Kreisberechnung',
    note: 'Circumference and area for a radius (integer maths, π ≈ 314/100).',
    listing: `10 PRINT "KREIS"
20 INPUT "RADIUS:"R
30 LET U=R*314/50
40 LET A=R*314/100*R
50 PRINT "UMFANG="U
60 PRINT "INHALT="A
70 PRINT "============="
80 END`,
  },
  {
    name: 'Mondlandung',
    note: 'Lunar landing: enter fuel (0–100) per step; land softly with fuel left.',
    listing: `10 PRINT "MONDLANDUNG"
20 LET V=90,H=900,T=500
30 PRINT "V="V
40 PRINT "H="H
50 PRINT "T="T
60 INPUT "EINSATZ:"E
70 IF E>100 THEN GOTO 60
80 IF T=0 THEN GOTO 130
90 IF T<E THEN GOTO 60
100 LET T=T-E,H=H-V-5+(E/2),V=V+10-E
110 IF H>0 THEN GOTO 30
120 LET H=ABS[V]/3-H
130 IF H>9 THEN PRINT "BRUCH";GOTO 160
140 IF H>4 THEN PRINT "HARTE";GOTO 160
150 PRINT "WEICHE"
160 PRINT "LANDUNG!";END`,
  },
]
