FasdUAS 1.101.10   ��   ��    k             l   � ����  O    �  	  k   � 
 
     l   ��  ��     activate     �    a c t i v a t e      r        m       �      o      ���� $0 xcodeprojektname XcodeProjektname      r        m    	   �      o      ���� 0 filecontents fileContents      r        4    ��  
�� 
alis   l    !���� ! l    "���� " I   �� # $
�� .earsffdralis        afdr #  f     $ �� %��
�� 
rtyp % m    ��
�� 
ctxt��  ��  ��  ��  ��    o      ���� 0 
homeordner     & ' & l   �� ( )��   ( 0 *display dialog "homeordner: " & homeordner    ) � * * T d i s p l a y   d i a l o g   " h o m e o r d n e r :   "   &   h o m e o r d n e r '  + , + l   ��������  ��  ��   ,  - . - r     / 0 / n     1 2 1 m    ��
�� 
ctnr 2 o    ���� 0 
homeordner   0 o      ���� 0 homeordnerpfad   .  3 4 3 l   �� 5 6��   5 2 ,set main to file "datum.c" of homeordnerpfad    6 � 7 7 X s e t   m a i n   t o   f i l e   " d a t u m . c "   o f   h o m e o r d n e r p f a d 4  8 9 8 r    & : ; : n    $ < = < 1   " $��
�� 
pnam = n    " > ? > 2     "��
�� 
file ? o     ���� 0 homeordnerpfad   ; o      ���� 0 dateienliste Dateienliste 9  @ A @ X   ' O B�� C B Z   9 J D E���� D E   9 > F G F o   9 :���� 0 tempname   G m   : = H H � I I  . x c o d e p r o j E k   A F J J  K L K r   A D M N M o   A B���� 0 tempname   N o      ���� $0 xcodeprojektname XcodeProjektname L  O�� O l  E E�� P Q��   P  display dialog tempname    Q � R R . d i s p l a y   d i a l o g   t e m p n a m e��  ��  ��  �� 0 tempname   C o   * +���� 0 dateienliste Dateienliste A  S T S l  P P��������  ��  ��   T  U V U r   P ] W X W b   P Y Y Z Y l  P U [���� [ c   P U \ ] \ o   P Q���� 0 homeordnerpfad   ] m   Q T��
�� 
TEXT��  ��   Z m   U X ^ ^ � _ _  d a t u m . c X o      ���� 0 filepfad   V  ` a ` I  ^ i�� b��
�� .sysodlogaskr        TEXT b b   ^ e c d c m   ^ a e e � f f  f i l e p f a d :   d o   a d���� 0 filepfad  ��   a  g h g l  j j�� i j��   i ! tell application "TextEdit"    j � k k 6 t e l l   a p p l i c a t i o n   " T e x t E d i t " h  l m l I  j o������
�� .miscactvnull��� ��� obj ��  ��   m  n o n r   p � p q p l  p ~ r���� r I  p ~�� s t
�� .rdwropenshor       file s 4   p v�� u
�� 
file u o   r u���� 0 filepfad   t �� v��
�� 
perm v m   y z��
�� boovtrue��  ��  ��   q o      ���� 0 refnum RefNum o  w x w Q   � y z { y k   � | |  } ~ } r   � �  �  l  � � ����� � I  � ��� ���
�� .rdwrread****        **** � o   � ����� 0 refnum RefNum��  ��  ��   � o      ���� 0 filecontents fileContents ~  � � � l  � ���������  ��  ��   �  � � � I  � ��� ���
�� .sysodlogaskr        TEXT � b   � � � � � b   � � � � � m   � � � � � � �  i n h a l t :   � o   � ���
�� 
ret  � o   � ����� 0 filecontents fileContents��   �  � � � r   � � � � � n   � � � � � 4   � ��� �
�� 
cpar � m   � �����  � o   � ����� 0 filecontents fileContents � o      ���� 0 datum Datum �  � � � I  � ��� ���
�� .sysodlogaskr        TEXT � b   � � � � � m   � � � � � � �  D a t u m :   � o   � ����� 0 datum Datum��   �  � � � r   � � � � � I  � �������
�� .misccurdldt    ��� null��  ��   � o      ���� 	0 heute   �  � � � I  � ��� ���
�� .sysodlogaskr        TEXT � b   � � � � � m   � � � � � � �  h e u t e :   � o   � ����� 	0 heute  ��   �  � � � r   � � � � � n   � � � � � 1   � ���
�� 
year � o   � ����� 	0 heute   � o      ���� 0 jahrtext   �  � � � r   � � � � � n   � � � � � m   � ���
�� 
mnth � o   � ����� 	0 heute   � o      ���� 0 	monattext   �  � � � I  � ��� ���
�� .sysodlogaskr        TEXT � b   � � � � � m   � � � � � � �  m o n a t :   � o   � ����� 0 	monattext  ��   �  � � � r   �
 � � � n   � � � � 7  ��� � �
�� 
ctxt � m   ������� � m  ������ � l  � � ����� � b   � � � � � m   � � � � � � �  0 � n   � � � � � 1   � ���
�� 
day  � o   � ����� 	0 heute  ��  ��   � o      ���� 0 tag   �  � � � l �� � ���   � " display dialog "tag: " & tag    � � � � 8 d i s p l a y   d i a l o g   " t a g :   "   &   t a g �  � � � r  7 � � � J  3 � �  � � � m  ��
�� 
jan  �  � � � m  ��
�� 
feb  �  � � � m  ��
�� 
mar  �  � � � l 	 ����� � m  ��
�� 
apr ��  ��   �  � � � m  ��
�� 
may  �  � � � m  ��
�� 
jun  �  � � � m   ��
�� 
jul  �  � � � m   #��
�� 
aug  �  � � � l 	#& ����� � m  #&��
�� 
sep ��  ��   �  � � � m  &)��
�� 
oct  �  � � � m  ),��
�� 
nov  �  ��� � m  ,/��
�� 
dec ��   � o      ���� 0 monatsliste MonatsListe �  � � � Y  8s ��� � ��� � Z  Dn � ����� � =  DP � � � o  DG�� 0 	monattext   � n  GO � � � 4  JO�~ �
�~ 
cobj � o  MN�}�} 0 i   � o  GJ�|�| 0 monatsliste MonatsListe � k  Sj � �  � � � r  Sh � � � n  Sd � � � 7 Xd�{ � 
�{ 
ctxt � m  \`�z�z��  m  ac�y�y�� � l SX�x�w b  SX m  SV �  0 o  VW�v�v 0 i  �x  �w   � o      �u�u 	0 monat   � �t l ij	  S  ij - ' wenn true, wird die Schleife verlassen   	 �

 N   w e n n   t r u e ,   w i r d   d i e   S c h l e i f e   v e r l a s s e n�t  ��  ��  �� 0 i   � m  ;<�s�s  � m  <?�r�r ��   �  l tt�q�q   &  display dialog "monat: " & monat    � @ d i s p l a y   d i a l o g   " m o n a t :   "   &   m o n a t  r  t� l 	t��p�o l t��n�m n t� 7 y��l
�l 
cha  m  ��k�k  m  ���j�j  l ty�i�h c  ty o  tw�g�g 0 jahrtext   m  wx�f
�f 
ctxt�i  �h  �n  �m  �p  �o   o      �e�e 0 jahr    l ���d �d   ? 9display dialog "jahr: " & jahr & " jahrtext: " & jahrtext     �!! r d i s p l a y   d i a l o g   " j a h r :   "   &   j a h r   &   "   j a h r t e x t :   "   &   j a h r t e x t "#" r  ��$%$ n  ��&'& m  ���c
�c 
nmbr' n  ��()( 2 ���b
�b 
cha ) o  ���a�a 0 datum Datum% o      �`�` 0 l  # *+* l ���_,-�_  , 1 +set neuesDatum to text -l thru -13 of Datum   - �.. V s e t   n e u e s D a t u m   t o   t e x t   - l   t h r u   - 1 3   o f   D a t u m+ /0/ l ��1231 r  ��454 n  ��676 7 ���^89
�^ 
ctxt8 m  ���]�] 9 m  ���\�\ 7 o  ���[�[ 0 datum Datum5 o      �Z�Z 0 
neuesdatum 
neuesDatum2 $  Anfang bis und mit Leerschlag   3 �:: <   A n f a n g   b i s   u n d   m i t   L e e r s c h l a g0 ;<; l ���Y=>�Y  = 2 ,display dialog "neuesDatum A: " & neuesDatum   > �?? X d i s p l a y   d i a l o g   " n e u e s D a t u m   A :   "   &   n e u e s D a t u m< @A@ r  ��BCB b  ��DED b  ��FGF b  ��HIH b  ��JKJ b  ��LML b  ��NON b  ��PQP o  ���X�X 0 
neuesdatum 
neuesDatumQ m  ��RR �SS  "O o  ���W�W 0 tag  M m  ��TT �UU  .K o  ���V�V 	0 monat  I m  ��VV �WW  .G o  ���U�U 0 jahrtext  E m  ��XX �YY  "C o      �T�T 0 
neuesdatum 
neuesDatumA Z[Z l ���S\]�S  \ 0 *display dialog "neuesDatum: " & neuesDatum   ] �^^ T d i s p l a y   d i a l o g   " n e u e s D a t u m :   "   &   n e u e s D a t u m[ _`_ r  ��aba b  ��cdc b  ��efe n  ��ghg 4  ���Ri
�R 
cpari m  ���Q�Q h o  ���P�P 0 filecontents fileContentsf o  ���O
�O 
ret d o  ���N�N 0 
neuesdatum 
neuesDatumb o      �M�M 0 	neuertext 	neuerText` jkj l ���Llm�L  l 3 -set paragraph 2 of fileContents to neuesDatum   m �nn Z s e t   p a r a g r a p h   2   o f   f i l e C o n t e n t s   t o   n e u e s D a t u mk opo l ���Kqr�K  q 9 3display dialog "neues Datum: " & return & neuerText   r �ss f d i s p l a y   d i a l o g   " n e u e s   D a t u m :   "   &   r e t u r n   &   n e u e r T e x tp tut I ���Jvw
�J .rdwrseofnull���     ****v o  ���I�I 0 refnum RefNumw �Hx�G
�H 
set2x m  ���F�F  �G  u yzy I � �E{|
�E .rdwrwritnull���     ****{ o  ���D�D 0 	neuertext 	neuerText| �C}�B
�C 
refn} o  ���A�A 0 refnum RefNum�B  z ~�@~ I �?�>
�? .rdwrclosnull���     **** o  �=�= 0 refnum RefNum�>  �@   z R      �<�;�:
�< .ascrerr ****      � ****�;  �:   { I �9��8
�9 .rdwrclosnull���     ****� o  �7�7 0 refnum RefNum�8   x ��� l �6�5�4�6  �5  �4  � ��� l �3���3  �   Neue Version einsetzen   � ��� .   N e u e   V e r s i o n   e i n s e t z e n� ��� r  ��� m  �� ���  � o      �2�2 0 filecontents fileContents� ��� l �1���1  � 4 .set homeordner to alias ((path to me as text))   � ��� \ s e t   h o m e o r d n e r   t o   a l i a s   ( ( p a t h   t o   m e   a s   t e x t ) )� ��� l �0���0  � 4 .set homeordner to alias ((path to me as text))   � ��� \ s e t   h o m e o r d n e r   t o   a l i a s   ( ( p a t h   t o   m e   a s   t e x t ) )� ��� l �/���/  � 2 ,display dialog "homeordner 2: " & homeordner   � ��� X d i s p l a y   d i a l o g   " h o m e o r d n e r   2 :   "   &   h o m e o r d n e r� ��� r  #��� n  !��� m  !�.
�. 
ctnr� o  �-�- 0 
homeordner  � o      �,�, 0 homeordnerpfad  � ��� r  $+��� n  $'��� 1  %'�+
�+ 
pnam� o  $%�*�* 0 homeordnerpfad  � o      �)�) 0 projektname Projektname� ��� l ,,�(���(  � 2 ,display dialog "Projektname: " & Projektname   � ��� X d i s p l a y   d i a l o g   " P r o j e k t n a m e :   "   &   P r o j e k t n a m e� ��� r  ,7��� n ,3��� 1  /3�'
�' 
txdl� 1  ,/�&
�& 
ascr� o      �%�% 0 olddels oldDels� ��� r  8C��� m  8;�� ���  _� n     ��� 1  >B�$
�$ 
txdl� 1  ;>�#
�# 
ascr� ��� l DD�"�!� �"  �!  �   � ��� r  DO��� n  DK��� 2 GK�
� 
citm� o  DG�� 0 projektname Projektname� o      �� 0 zeilenliste Zeilenliste� ��� r  P[��� n  PW��� m  SW�
� 
nmbr� o  PS�� 0 zeilenliste Zeilenliste� o      �� 0 	anzzeilen 	anzZeilen� ��� l \\����  � n hdisplay dialog "Zeilenliste: " & return & (Zeilenliste as list) & return & "Anzahl Zeilen: " & anzZeilen   � ��� � d i s p l a y   d i a l o g   " Z e i l e n l i s t e :   "   &   r e t u r n   &   ( Z e i l e n l i s t e   a s   l i s t )   &   r e t u r n   &   " A n z a h l   Z e i l e n :   "   &   a n z Z e i l e n� ��� l \\����  �  �  � ��� l \\����  � � �display dialog "Zeilenliste: " & return & item 1 of Zeilenliste & return & item 2 of Zeilenliste & return & item 3 of Zeilenliste & return --& item 4 of Zeilenliste & return & item 5 of Zeilenliste   � ���� d i s p l a y   d i a l o g   " Z e i l e n l i s t e :   "   &   r e t u r n   &   i t e m   1   o f   Z e i l e n l i s t e   &   r e t u r n   &   i t e m   2   o f   Z e i l e n l i s t e   &   r e t u r n   &   i t e m   3   o f   Z e i l e n l i s t e   &   r e t u r n   - - &   i t e m   4   o f   Z e i l e n l i s t e   &   r e t u r n   &   i t e m   5   o f   Z e i l e n l i s t e� ��� r  \l��� n  \h��� 4  _h��
� 
cobj� l bg���� \  bg��� o  be�� 0 	anzzeilen 	anzZeilen� m  ef�� �  �  � o  \_�� 0 zeilenliste Zeilenliste� o      �� 0 version1 Version1� ��� r  m{��� n  mw��� 4  pw��
� 
cobj� o  sv�� 0 	anzzeilen 	anzZeilen� o  mp�� 0 zeilenliste Zeilenliste� o      �
�
 0 version2 Version2� ��� l ||�	���	  �  �  � ��� r  |���� o  |�� 0 olddels oldDels� n     ��� 1  ���
� 
txdl� 1  ��
� 
ascr� ��� l ������  �  �  � ��� l ��� ���   � 2 ,set main to file "datum.c" of homeordnerpfad   � ��� X s e t   m a i n   t o   f i l e   " d a t u m . c "   o f   h o m e o r d n e r p f a d� ��� r  ����� b  ����� l �������� c  ����� o  ������ 0 homeordnerpfad  � m  ����
�� 
TEXT��  ��  � m  ���� �    v e r s i o n . c� o      ���� 0 filepfad  �  l ������   , &display dialog "filepfad: " & filepfad    � L d i s p l a y   d i a l o g   " f i l e p f a d :   "   &   f i l e p f a d  r  ��	 b  ��

 b  �� b  �� b  �� b  �� b  �� b  �� b  �� b  �� m  �� �  / m  �� �  / m  ��   �!!  v e r s i o n . c o  ����
�� 
ret  m  ��"" �##   # d e f i n e   V E R S I O N   m  ��$$ �%%  " o  ������ 0 version1 Version1 m  ��&& �''  . o  ������ 0 version2 Version2 m  ��(( �))  "	 o      ���� 0 
erstertext 
ersterText *+* l ����,-��  , : 4display dialog "erster Text: " & return & ersterText   - �.. h d i s p l a y   d i a l o g   " e r s t e r   T e x t :   "   &   r e t u r n   &   e r s t e r T e x t+ /0/ l ����������  ��  ��  0 121 l ����������  ��  ��  2 343 I ��������
�� .miscactvnull��� ��� obj ��  ��  4 565 Q  ��7897 k  �w:: ;<; l ����������  ��  ��  < =>= r  ��?@? l ��A����A I ����BC
�� .rdwropenshor       fileB 4  ����D
�� 
fileD o  ������ 0 filepfad  C ��E��
�� 
permE m  ����
�� boovtrue��  ��  ��  @ o      ���� 0 refnum RefNum> FGF r  ��HIH l ��J����J I ����K��
�� .rdwrread****        ****K o  ������ 0 refnum RefNum��  ��  ��  I o      ���� 0 filecontents fileContentsG LML I ����N��
�� .sysodlogaskr        TEXTN b  ��OPO b  ��QRQ m  ��SS �TT  i n h a l t :  R o  ����
�� 
ret P o  ������ 0 filecontents fileContents��  M UVU l ����������  ��  ��  V WXW r  � YZY n  ��[\[ 4  ����]
�� 
cpar] m  ������ \ o  ������ 0 filecontents fileContentsZ o      ���� 0 alteversion  X ^_^ I ��`��
�� .sysodlogaskr        TEXT` b  aba m  cc �dd  a l t e   V e r s i o n :  b o  ���� 0 alteversion  ��  _ efe r  ghg n  iji m  ��
�� 
nmbrj n  klk 2 ��
�� 
cha l o  ���� 0 alteversion  h o      ���� 0 l  f mnm l 0opqo r  0rsr n  ,tut 7  ,��vw
�� 
ctxtv m  $&���� w m  '+���� u o   ���� 0 alteversion  s o      ���� 0 neueversion neueVersionp $  Anfang bis und mit Leerschlag   q �xx <   A n f a n g   b i s   u n d   m i t   L e e r s c h l a gn yzy l 1J{|}{ r  1J~~ n  1F��� 7 4F����
�� 
ctxt� l 8>������ \  8>��� o  9<���� 0 l  � m  <=���� ��  ��  � l ?E������ \  ?E��� o  @C���� 0 l  � m  CD���� ��  ��  � o  14���� 0 alteversion   o      ���� &0 alteversionnummer alteVersionnummer|      } ���   z ��� I Kb�����
�� .sysodlogaskr        TEXT� b  K^��� b  KV��� b  KR��� m  KN�� ��� & a l t e V e r s i o n n u m m e r :  � o  NQ���� &0 alteversionnummer alteVersionnummer� m  RU�� ���    a l s   i n t e g e r :  � l V]������ c  V]��� o  VY���� &0 alteversionnummer alteVersionnummer� m  Y\��
�� 
long��  ��  ��  � ��� l cc��������  ��  ��  � ��� r  c���� I c�����
�� .sysodlogaskr        TEXT� m  cf�� ���  V e r s i o n u m m e r :� ����
�� 
dtxt� l 	il������ o  il���� &0 alteversionnummer alteVersionnummer��  ��  � ����
�� 
btns� l 	oz������ J  oz�� ��� m  or�� ���  O K� ��� m  ru�� ���  I n c r e m e n t� ���� m  ux�� ���  n e u   s e t z e n��  ��  ��  � ����
�� 
dflt� l 
}~������ m  }~���� ��  ��  � �����
�� 
disp� m  ����
�� stic   ��  � o      ���� 0 antwort  � ��� l ����������  ��  ��  � ��� l ��������  � # set DialogResultat to result    � ��� : s e t   D i a l o g R e s u l t a t   t o   r e s u l t  � ��� l ��������  � � �display dialog "button returned: " & button returned of result --& "text: " & text returned of antwort & "resultat: " & DialogResultat --& "antwort: " & antwort   � ���@ d i s p l a y   d i a l o g   " b u t t o n   r e t u r n e d :   "   &   b u t t o n   r e t u r n e d   o f   r e s u l t   - - &   " t e x t :   "   &   t e x t   r e t u r n e d   o f   a n t w o r t   &   " r e s u l t a t :   "   &   D i a l o g R e s u l t a t   - - &   " a n t w o r t :   "   &   a n t w o r t� ��� l ����������  ��  ��  � ��� l ��������  � . (if button returned of result = "OK" then   � ��� P i f   b u t t o n   r e t u r n e d   o f   r e s u l t   =   " O K "   t h e n� ��� r  ����� o  ������ &0 alteversionnummer alteVersionnummer� o      ���� 0 versionnummer versionNummer� ��� l ��������  �  end if   � ���  e n d   i f� ��� Z  �&������ = ����� n  ����� 1  ����
�� 
bhit� o  ������ 0 antwort  � m  ���� ���  I n c r e m e n t� k  ��� ��� l ��������  �  display dialog "erh�hen"   � ��� 0 d i s p l a y   d i a l o g   " e r h � h e n "� ��� r  ����� [  ����� l �������� c  ����� o  ������ &0 alteversionnummer alteVersionnummer� m  ����
�� 
long��  ��  � m  ������ � o      ���� &0 neueversionnummer neueVersionnummer� ��� Z  �������� l �������� A  ����� o  ������ &0 neueversionnummer neueVersionnummer� m  ������ 
��  ��  � r  ����� b  ����� m  ���� ���  0 0� l �������� c  ��   o  ���� &0 neueversionnummer neueVersionnummer m  ���~
�~ 
ctxt��  ��  � o      �}�} &0 neueversionnummer neueVersionnummer�  l ���|�{ A  �� o  ���z�z &0 neueversionnummer neueVersionnummer m  ���y�y d�|  �{    r  ��	
	 b  �� m  �� �  0 o  ���x�x &0 neueversionnummer neueVersionnummer
 o      �w�w &0 neueversionnummer neueVersionnummer  l ���v�u ?  �� o  ���t�t &0 neueversionnummer neueVersionnummer m  ���s�s ��v  �u   �r r  �� m  �� �  0 0 1 o      �q�q &0 neueversionnummer neueVersionnummer�r  ��  �  r  � o  ���p�p &0 neueversionnummer neueVersionnummer o      �o�o 0 versionnummer versionNummer �n l �m�m   = 7display dialog "neue Versionummer:" & neueVersionnummer    �   n d i s p l a y   d i a l o g   " n e u e   V e r s i o n u m m e r : "   &   n e u e V e r s i o n n u m m e r�n  � !"! = #$# n  %&% 1  
�l
�l 
bhit& o  
�k�k 0 antwort  $ m  '' �((  n e u   s e t z e n" )�j) k  "** +,+ l �i-.�i  - G Adisplay dialog "neu setzen: 3" & (text returned of antwort) & "*"   . �// � d i s p l a y   d i a l o g   " n e u   s e t z e n :   3 "   &   ( t e x t   r e t u r n e d   o f   a n t w o r t )   &   " * ", 010 r   232 l 4�h�g4 n  565 1  �f
�f 
ttxt6 o  �e�e 0 antwort  �h  �g  3 o      �d�d 0 versionnummer versionNummer1 7�c7 l !!�b89�b  8 B <display dialog "neue gesetzte Versionummer:" & versionNummer   9 �:: x d i s p l a y   d i a l o g   " n e u e   g e s e t z t e   V e r s i o n u m m e r : "   &   v e r s i o n N u m m e r�c  �j  ��  � ;<; l ''�a�`�_�a  �`  �_  < =>= l ''�^?@�^  ?   set versionNummer to "002"   @ �AA 4 s e t   v e r s i o n N u m m e r   t o   " 0 0 2 "> BCB l ''�]�\�[�]  �\  �[  C DED r  'UFGF b  'QHIH b  'MJKJ b  'ILML b  'ENON b  'APQP b  '=RSR b  '9TUT b  '5VWV b  '1XYX n  '-Z[Z 4  (-�Z\
�Z 
cpar\ m  +,�Y�Y [ o  '(�X�X 0 filecontents fileContentsY o  -0�W
�W 
ret W o  14�V�V 0 neueversion neueVersionU m  58]] �^^  "S o  9<�U�U 0 version1 Version1Q m  =@__ �``  .O o  AD�T�T 0 version2 Version2M m  EHaa �bb  .K o  IL�S�S 0 versionnummer versionNummerI m  MPcc �dd  "G o      �R�R 0 	neuertext 	neuerTextE efe l VV�Qgh�Q  g 4 .set paragraph 2 of fileContents to neueVersion   h �ii \ s e t   p a r a g r a p h   2   o f   f i l e C o n t e n t s   t o   n e u e V e r s i o nf jkj l VV�Plm�P  l : 4display dialog "neue Version: " & return & neuerText   m �nn h d i s p l a y   d i a l o g   " n e u e   V e r s i o n :   "   &   r e t u r n   &   n e u e r T e x tk opo l VV�Oqr�O  q  close access RefNum   r �ss & c l o s e   a c c e s s   R e f N u mp tut l VV�Nvw�N  v  return   w �xx  r e t u r nu yzy l VV�M�L�K�M  �L  �K  z {|{ I Va�J}~
�J .rdwrseofnull���     ****} o  VY�I�I 0 refnum RefNum~ �H�G
�H 
set2 m  \]�F�F  �G  | ��� I bo�E��
�E .rdwrwritnull���     ****� o  be�D�D 0 	neuertext 	neuerText� �C��B
�C 
refn� o  hk�A�A 0 refnum RefNum�B  � ��@� I pw�?��>
�? .rdwrclosnull���     ****� o  ps�=�= 0 refnum RefNum�>  �@  8 R      �<��
�< .ascrerr ****      � ****� o      �;�; 0 errmsg errMsg� �:��9
�: 
errn� o      �8�8 0 errnr errNr�9  9 k  ��� ��� I ��7��6
�7 .ascrcmnt****      � ****� o  ��5�5 0 errnr errNr�6  � ��� Q  ������ k  ���� ��� I ���4��3
�4 .sysobeepnull��� ��� long� J  ���2�2  �3  � ��� l ���1���1  � w qset ersterText to "/" & "/" & "version.c" & return & "#define VERSION " & "\"" & Version1 & "." & Version2 & "\""   � ��� � s e t   e r s t e r T e x t   t o   " / "   &   " / "   &   " v e r s i o n . c "   &   r e t u r n   &   " # d e f i n e   V E R S I O N   "   &   " \ " "   &   V e r s i o n 1   &   " . "   &   V e r s i o n 2   &   " \ " "� ��� l ���0���0  � : 4display dialog "erstes  File: " & return & neuerText   � ��� h d i s p l a y   d i a l o g   " e r s t e s     F i l e :   "   &   r e t u r n   &   n e u e r T e x t� ��/� I ���.��-
�. .rdwrclosnull���     ****� o  ���,�, 0 refnum RefNum�-  �/  � R      �+��
�+ .ascrerr ****      � ****� o      �*�* 0 errmsg errMsg� �)��(
�) 
errn� o      �'�' 0 errnr errNr�(  � k  ���� ��� l ���&���&  �  errMsg --number errNr   � ��� * e r r M s g   - - n u m b e r   e r r N r� ��� l ���%���%  �  	log errNr   � ���  l o g   e r r N r� ��� I ���$��#
�$ .ascrcmnt****      � ****� o  ���"�" 0 errmsg errMsg�#  � ��!� l ��� ���   �  close access RefNum   � ��� & c l o s e   a c c e s s   R e f N u m�!  � ��� l ������  �  close access RefNum   � ��� & c l o s e   a c c e s s   R e f N u m�  6 ��� l ������  �  �  � ��� n  ����� I  ������ $0 logaktualisieren LogAktualisieren� ��� o  ���� 0 
homeordner  �  �  �  f  ��� ��� l ������  �  �  � ��� I �����
� .aevtodocnull  �    alis� n  ����� 4  ����
� 
file� o  ���� $0 xcodeprojektname XcodeProjektname� o  ���� 0 homeordnerpfad  �  �   	 m     ���                                                                                  MACS  alis    @  Macintosh HD                   BD ����
Finder.app                                                     ����            ����  
 cu             CoreServices  )/:System:Library:CoreServices:Finder.app/    
 F i n d e r . a p p    M a c i n t o s h   H D  &System/Library/CoreServices/Finder.app  / ��  ��  ��    ��� l     ����  �  �  � ��
� i     ��� I      �	���	 $0 logaktualisieren LogAktualisieren� ��� o      �� 0 
homeordner  �  �  � O    ���� k   ��� ��� I   	���
� .miscactvnull��� ��� obj �  �  � ��� l  
 
��� �  �  �   � ��� r   
 ��� m   
 �� ���  � o      ���� 0 filecontents fileContents� ��� l   ������  � 4 .set homeordner to alias ((path to me as text))   � ��� \ s e t   h o m e o r d n e r   t o   a l i a s   ( ( p a t h   t o   m e   a s   t e x t ) )� ��� l   ������  � 0 *display dialog "homeordner: " & homeordner   � ��� T d i s p l a y   d i a l o g   " h o m e o r d n e r :   "   &   h o m e o r d n e r� ��� r    ��� n    ��� m    ��
�� 
ctnr� o    ���� 0 
homeordner  � o      ���� 0 homeordnerpfad  � ��� l   ������  �  open homeordnerpfad   � ��� & o p e n   h o m e o r d n e r p f a d� ��� l   ������  � 8 2display dialog "homeordnerpfad: " & homeordnerpfad   � ��� d d i s p l a y   d i a l o g   " h o m e o r d n e r p f a d :   "   &   h o m e o r d n e r p f a d� ��� l   ������  � 2 ,set main to file "datum.c" of homeordnerpfad   � ��� X s e t   m a i n   t o   f i l e   " d a t u m . c "   o f   h o m e o r d n e r p f a d� � � r     b     l   ���� c     o    ���� 0 homeordnerpfad   m    ��
�� 
TEXT��  ��   m     �		  L o g f i l e . t x t o      ���� 0 filepfad    

 l   ����   , &display dialog "filepfad: " & filepfad    � L d i s p l a y   d i a l o g   " f i l e p f a d :   "   &   f i l e p f a d  l   ��������  ��  ��    I   !������
�� .miscactvnull��� ��� obj ��  ��    r   " ) I  " '������
�� .misccurdldt    ��� null��  ��   o      ���� 	0 heute    l  * *����   &  display dialog "heute: " & heute    � @ d i s p l a y   d i a l o g   " h e u t e :   "   &   h e u t e  r   * / n   * - !  1   + -��
�� 
year! o   * +���� 	0 heute   o      ���� 0 jahrtext   "#" r   0 5$%$ n   0 3&'& m   1 3��
�� 
mnth' o   0 1���� 	0 heute  % o      ���� 0 	monattext  # ()( l  6 6��*+��  * * $display dialog "monat: " & monattext   + �,, H d i s p l a y   d i a l o g   " m o n a t :   "   &   m o n a t t e x t) -.- r   6 G/0/ n   6 E121 7  ; E��34
�� 
ctxt3 m   ? A������4 m   B D������2 l  6 ;5����5 b   6 ;676 m   6 788 �99  07 n   7 ::;: 1   8 :��
�� 
day ; o   7 8���� 	0 heute  ��  ��  0 o      ���� 0 tag  . <=< l  H H��>?��  > " display dialog "tag: " & tag   ? �@@ 8 d i s p l a y   d i a l o g   " t a g :   "   &   t a g= ABA r   H lCDC J   H jEE FGF m   H I��
�� 
jan G HIH m   I J��
�� 
feb I JKJ m   J K��
�� 
mar K LML l 	 K NN����N m   K N��
�� 
apr ��  ��  M OPO m   N Q��
�� 
may P QRQ m   Q T��
�� 
jun R STS m   T W��
�� 
jul T UVU m   W Z��
�� 
aug V WXW l 	 Z ]Y����Y m   Z ]��
�� 
sep ��  ��  X Z[Z m   ] `��
�� 
oct [ \]\ m   ` c��
�� 
nov ] ^��^ m   c f��
�� 
dec ��  D o      ���� 0 monatsliste MonatsListeB _`_ Y   m �a��bc��a Z   y �de����d =   y �fgf o   y z���� 0 	monattext  g n   z �hih 4   { ���j
�� 
cobjj o   ~ ���� 0 i  i o   z {���� 0 monatsliste MonatsListee k   � �kk lml r   � �non n   � �pqp 7  � ���rs
�� 
ctxtr m   � �������s m   � �������q l  � �t����t b   � �uvu m   � �ww �xx  0v o   � ����� 0 i  ��  ��  o o      ���� 	0 monat  m y��y l  � �z{|z  S   � �{ - ' wenn true, wird die Schleife verlassen   | �}} N   w e n n   t r u e ,   w i r d   d i e   S c h l e i f e   v e r l a s s e n��  ��  ��  �� 0 i  b m   p q���� c m   q t���� ��  ` ~~ l  � �������  � &  display dialog "monat: " & monat   � ��� @ d i s p l a y   d i a l o g   " m o n a t :   "   &   m o n a t ��� r   � ���� l 	 � ������� l  � ������� n  � ���� 7  � �����
�� 
cha � m   � ����� � m   � ����� � l  � ������� c   � ���� o   � ����� 0 jahrtext  � m   � ���
�� 
ctxt��  ��  ��  ��  ��  ��  � o      ���� 0 jahr  � ��� l  � �������  � ? 9display dialog "jahr: " & jahr & " jahrtext: " & jahrtext   � ��� r d i s p l a y   d i a l o g   " j a h r :   "   &   j a h r   &   "   j a h r t e x t :   "   &   j a h r t e x t� ��� l  � �������  � , &set l to number of characters of Datum   � ��� L s e t   l   t o   n u m b e r   o f   c h a r a c t e r s   o f   D a t u m� ��� l  � �������  � 1 +set neuesDatum to text -l thru -13 of Datum   � ��� V s e t   n e u e s D a t u m   t o   t e x t   - l   t h r u   - 1 3   o f   D a t u m� ��� l  � �������  � P Jset neuesDatum to text 1 thru 14 of Datum -- Anfang bis und mit Leerschlag   � ��� � s e t   n e u e s D a t u m   t o   t e x t   1   t h r u   1 4   o f   D a t u m   - -   A n f a n g   b i s   u n d   m i t   L e e r s c h l a g� ��� r   � ���� b   � ���� b   � ���� b   � ���� b   � ���� o   � ����� 0 tag  � m   � ��� ���  .� o   � ����� 	0 monat  � m   � ��� ���  .� o   � ����� 0 jahrtext  � o      ���� 0 
neuesdatum 
neuesDatum� ��� l  � �������  � 0 *display dialog "neuesDatum: " & neuesDatum   � ��� T d i s p l a y   d i a l o g   " n e u e s D a t u m :   "   &   n e u e s D a t u m� ��� l  � ���������  ��  ��  � ��� l  � ���������  ��  ��  � ��� r   � ���� l  � ������� I  � �����
�� .rdwropenshor       file� 4   � ����
�� 
file� o   � ����� 0 filepfad  � �����
�� 
perm� m   � ���
�� boovtrue��  ��  ��  � o      ���� 0 refnum RefNum� ��� Q   �A���� k   �4�� ��� r   � ���� l  � ������� I  � ������
�� .rdwrread****        ****� o   � ����� 0 refnum RefNum��  ��  ��  � o      ���� 0 filecontents fileContents� ��� r   � ���� n   � ���� 4  � ����
�� 
cwor� m   � �������� l  � ������� n   � ���� 4   � ����
�� 
cpar� m   � ����� � o   � ����� 0 filecontents fileContents��  ��  � o      �� 0 	lastdatum 	lastDatum� ��� l  � ��~���~  � 7 1display dialog "lastDatum: " & return & lastDatum   � ��� b d i s p l a y   d i a l o g   " l a s t D a t u m :   "   &   r e t u r n   &   l a s t D a t u m� ��� l  � ��}���}  � . (set Datum to paragraph 2 of fileContents   � ��� P s e t   D a t u m   t o   p a r a g r a p h   2   o f   f i l e C o n t e n t s� ��� l  � ��|���|  � &  display dialog "Datum: " & Datum   � ��� @ d i s p l a y   d i a l o g   " D a t u m :   "   &   D a t u m� ��� Z   �.���{�� =  � ���� o   � ��z�z 0 
neuesdatum 
neuesDatum� o   � ��y�y 0 	lastdatum 	lastDatum� l  � ��x���x  � % display dialog "gleiches Datum"   � ��� > d i s p l a y   d i a l o g   " g l e i c h e s   D a t u m "�{  � k   �.�� ��� l  � ��w�v�u�w  �v  �u  � ��� r   ���� b   ���� b   �� � b   � b   � b   �
 b   � b   �	
	 b   � � m   � � � T * * * * * * * * * * * * * * * * * * * * * *                                         o   � ��t�t 0 
neuesdatum 
neuesDatum
 o   ��s
�s 
ret  l 	�r�q o  �p
�p 
ret �r  �q   o  	�o
�o 
ret  l 	
�n�m m  
 � , * * * * * * * * * * * * * * * * * * * * * *�n  �m   o  �l
�l 
ret   o  �k�k 0 filecontents fileContents� o  �j
�j 
ret � o      �i�i 0 	neuertext 	neuerText�  I $�h
�h .rdwrseofnull���     **** o  �g�g 0 refnum RefNum �f�e
�f 
set2 m   �d�d  �e   �c I %.�b
�b .rdwrwritnull���     **** o  %&�a�a 0 	neuertext 	neuerText �`�_
�` 
refn o  )*�^�^ 0 refnum RefNum�_  �c  � �] I /4�\�[
�\ .rdwrclosnull���     **** o  /0�Z�Z 0 refnum RefNum�[  �]  � R      �Y�X�W
�Y .ascrerr ****      � ****�X  �W  � k  <A   l <<�V�U�T�V  �U  �T    !�S! I <A�R"�Q
�R .rdwrclosnull���     ****" o  <=�P�P 0 refnum RefNum�Q  �S  � #$# l BB�O%&�O  %  start   & �'' 
 s t a r t$ ()( r  BK*+* J  BG,, -�N- m  BE.. �//  x c o d e p r o j�N  + o      �M�M 0 filetype  ) 010 l LL�L23�L  2 ? 9set projektpfad to (path to alias (homeordner)) as string   3 �44 r s e t   p r o j e k t p f a d   t o   ( p a t h   t o   a l i a s   ( h o m e o r d n e r ) )   a s   s t r i n g1 565 l LL�K78�K  7 0 *display dialog "projektpfad" & projektpfad   8 �99 T d i s p l a y   d i a l o g   " p r o j e k t p f a d "   &   p r o j e k t p f a d6 :;: l LL�J<=�J  < 8 2display dialog "homeordnerpfad: " & homeordnerpfad   = �>> d d i s p l a y   d i a l o g   " h o m e o r d n e r p f a d :   "   &   h o m e o r d n e r p f a d; ?@? l LL�IAB�I  A > 8get name of folders of folder (homeordnerpfad as string)   B �CC p g e t   n a m e   o f   f o l d e r s   o f   f o l d e r   ( h o m e o r d n e r p f a d   a s   s t r i n g )@ DED l L`FGHF r  L`IJI n  L\KLK 1  X\�H
�H 
pnamL n  LXMNM 2 TX�G
�G 
fileN 4  LT�FO
�F 
cfolO l PSP�E�DP c  PSQRQ o  PQ�C�C 0 homeordnerpfad  R m  QR�B
�B 
TEXT�E  �D  J o      �A�A 
0 inhalt  G  without invisibles   H �SS $ w i t h o u t   i n v i s i b l e sE TUT l aa�@VW�@  V # display dialog inhalt as text   W �XX : d i s p l a y   d i a l o g   i n h a l t   a s   t e x tU YZY l aa�?[\�?  [ 7 1repeat with i from 1 to number of items of inhalt   \ �]] b r e p e a t   w i t h   i   f r o m   1   t o   n u m b e r   o f   i t e m s   o f   i n h a l tZ ^�>^ X  a�_�=`_ k  w�aa bcb l ww�<de�<  d &  display dialog (dasFile) as text   e �ff @ d i s p l a y   d i a l o g   ( d a s F i l e )   a s   t e x tc g�;g Z  w�hi�:�9h E  w~jkj l wzl�8�7l l wzm�6�5m o  wz�4�4 0 dasfile dasFile�6  �5  �8  �7  k m  z}nn �oo  x c o d e p r o ji k  ��pp qrq r  ��sts b  ��uvu l ��w�3�2w c  ��xyx o  ���1�1 0 homeordnerpfad  y m  ���0
�0 
ctxt�3  �2  v l ��z�/�.z c  ��{|{ o  ���-�- 0 dasfile dasFile| m  ���,
�, 
ctxt�/  �.  t o      �+�+ 0 filepfad  r }~} l ���*��*   &  display dialog (dasFile) as text   � ��� @ d i s p l a y   d i a l o g   ( d a s F i l e )   a s   t e x t~ ��)� I ���(��'
�( .aevtodocnull  �    alis� 4  ���&�
�& 
file� o  ���%�% 0 filepfad  �'  �)  �:  �9  �;  �= 0 dasfile dasFile` o  dg�$�$ 
0 inhalt  �>  � m     ���                                                                                  MACS  alis    @  Macintosh HD                   BD ����
Finder.app                                                     ����            ����  
 cu             CoreServices  )/:System:Library:CoreServices:Finder.app/    
 F i n d e r . a p p    M a c i n t o s h   H D  &System/Library/CoreServices/Finder.app  / ��  �
       �#����#  � �"�!�" $0 logaktualisieren LogAktualisieren
�! .aevtoappnull  �   � ****� � �������  $0 logaktualisieren LogAktualisieren� ��� �  �� 0 
homeordner  �  � �����������������
�	�� 0 
homeordner  � 0 filecontents fileContents� 0 homeordnerpfad  � 0 filepfad  � 	0 heute  � 0 jahrtext  � 0 	monattext  � 0 tag  � 0 monatsliste MonatsListe� 0 i  � 	0 monat  � 0 jahr  � 0 
neuesdatum 
neuesDatum� 0 refnum RefNum� 0 	lastdatum 	lastDatum� 0 	neuertext 	neuerText�
 0 filetype  �	 
0 inhalt  � 0 dasfile dasFile� 7��������8�� ������������������������������w����������������������������������.��������n��
� .miscactvnull��� ��� obj 
� 
ctnr
� 
TEXT
� .misccurdldt    ��� null
� 
year
� 
mnth
� 
day 
�  
ctxt����
�� 
jan 
�� 
feb 
�� 
mar 
�� 
apr 
�� 
may 
�� 
jun 
�� 
jul 
�� 
aug 
�� 
sep 
�� 
oct 
�� 
nov 
�� 
dec �� 
�� 
cobj
�� 
cha �� 
�� 
file
�� 
perm
�� .rdwropenshor       file
�� .rdwrread****        ****
�� 
cpar
�� 
cwor
�� 
ret 
�� 
set2
�� .rdwrseofnull���     ****
�� 
refn
�� .rdwrwritnull���     ****
�� .rdwrclosnull���     ****��  ��  
�� 
cfol
�� 
pnam
�� 
kocl
�� .corecnte****       ****
�� .aevtodocnull  �    alis����*j O�E�O��,E�O��&�%E�O*j O*j E�O��,E�O��,E�O��,%[�\[Z�\Zi2E�O���a a a a a a a a a a vE�O 2ka kh 	��a �/  a �%[�\[Z�\Zi2E�OY h[OY��O��&[a \[Zm\Za 2E�O�a %�%a %�%E�O*a  �/a !el "E�O `�j #E�O�a $k/a %i/E�O��  hY 7a &�%_ '%_ '%_ '%a (%_ '%�%_ '%E�O�a )jl *O�a +�l ,O�j -W X . /�j -Oa 0kvE^ O*a 1��&/a  -a 2,E^ O >] [a 3a l 4kh ] a 5 ��&] �&%E�O*a  �/j 6Y h[OY��U� �����������
�� .aevtoappnull  �   � ****� k    ���  ����  ��  ��  � ���������� 0 tempname  �� 0 i  �� 0 errmsg errMsg�� 0 errnr errNr� �� �� ���������������������������� H�� ^�� e������������ ������� ����� ��������� � ���������������������������������������������������RTVX������������������������������������� "$&(��S��c����������������������~�}�|�{�z��y�x��w�v'�u]_ac�t��s�r�q�p�� $0 xcodeprojektname XcodeProjektname�� 0 filecontents fileContents
�� 
alis
�� 
rtyp
�� 
ctxt
�� .earsffdralis        afdr�� 0 
homeordner  
�� 
ctnr�� 0 homeordnerpfad  
�� 
file
�� 
pnam�� 0 dateienliste Dateienliste
�� 
kocl
�� 
cobj
�� .corecnte****       ****
�� 
TEXT�� 0 filepfad  
�� .sysodlogaskr        TEXT
�� .miscactvnull��� ��� obj 
�� 
perm
�� .rdwropenshor       file�� 0 refnum RefNum
�� .rdwrread****        ****
�� 
ret 
�� 
cpar�� 0 datum Datum
�� .misccurdldt    ��� null�� 	0 heute  
�� 
year�� 0 jahrtext  
�� 
mnth�� 0 	monattext  
�� 
day ������ 0 tag  
�� 
jan 
�� 
feb 
�� 
mar 
�� 
apr 
�� 
may 
�� 
jun 
�� 
jul 
�� 
aug 
�� 
sep 
�� 
oct 
�� 
nov 
�� 
dec �� �� 0 monatsliste MonatsListe�� 	0 monat  
�� 
cha �� �� 0 jahr  
�� 
nmbr�� 0 l  �� �� 0 
neuesdatum 
neuesDatum�� 0 	neuertext 	neuerText
�� 
set2
�� .rdwrseofnull���     ****
�� 
refn
�� .rdwrwritnull���     ****
�� .rdwrclosnull���     ****��  ��  �� 0 projektname Projektname
�� 
ascr
�� 
txdl�� 0 olddels oldDels
�� 
citm�� 0 zeilenliste Zeilenliste�� 0 	anzzeilen 	anzZeilen�� 0 version1 Version1�� 0 version2 Version2�� 0 
erstertext 
ersterText�� 0 alteversion  �� �� 0 neueversion neueVersion�� &0 alteversionnummer alteVersionnummer
�� 
long
�� 
dtxt
�� 
btns
�� 
dflt
� 
disp
�~ stic   �} �| 0 antwort  �{ 0 versionnummer versionNummer
�z 
bhit�y &0 neueversionnummer neueVersionnummer�x 
�w d�v �
�u 
ttxt�t 0 errmsg errMsg� �o�n�m
�o 
errn�n 0 errnr errNr�m  
�s .ascrcmnt****      � ****
�r .sysobeepnull��� ��� long�q $0 logaktualisieren LogAktualisieren
�p .aevtodocnull  �    alis������E�O�E�O*�)��l /E�O��,E�O��-�,E�O '�[�a l kh  �a  
�E�OPY h[OY��O�a &a %E` Oa _ %j O*j O*�_ /a el E` O�_ j E�Oa _ %�%j O�a l/E`  Oa !_  %j O*j "E` #Oa $_ #%j O_ #a %,E` &O_ #a ',E` (Oa )_ (%j Oa *_ #a +,%[�\[Za ,\Zi2E` -Oa .a /a 0a 1a 2a 3a 4a 5a 6a 7a 8a 9a :vE` ;O :ka :kh _ (_ ;a �/  a <�%[�\[Za ,\Zi2E` =OY h[OY��O_ &�&[a >\[Zm\Za ?2E` @O_  a >-a A,E` BO_  [�\[Zk\Za C2E` DO_ Da E%_ -%a F%_ =%a G%_ &%a H%E` DO�a k/_ %_ D%E` IO_ a Jjl KO_ Ia L_ l MO_ j NW X O P_ j NOa QE�O��,E�O��,E` RO_ Sa T,E` UOa V_ Sa T,FO_ Ra W-E` XO_ Xa A,E` YO_ Xa _ Yk/E` ZO_ Xa _ Y/E` [O_ U_ Sa T,FO�a &a \%E` Oa ]a ^%a _%_ %a `%a a%_ Z%a b%_ [%a c%E` dO*j O�*�_ /a el E` O_ j E�Oa e_ %�%j O�a l/E` fOa g_ f%j O_ fa >-a A,E` BO_ f[�\[Zk\Za h2E` iO_ f[�\[Z_ Bm\Z_ Bk2E` jOa k_ j%a l%_ ja m&%j Oa na o_ ja pa qa ra smva tka ua va w E` xO_ jE` yO_ xa z,a {  d_ ja m&kE` |O_ |a } a ~_ |�&%E` |Y /_ |a  a �_ |%E` |Y _ |a � a �E` |Y hO_ |E` yOPY !_ xa z,a �  _ xa �,E` yOPY hO�a k/_ %_ i%a �%_ Z%a �%_ [%a �%_ y%a �%E` IO_ a Jjl KO_ Ia L_ l MO_ j NW /X � ��j �O jvj �O_ j NW X � ��j �OPOPO)�k+ �O���/j �U ascr  ��ޭ