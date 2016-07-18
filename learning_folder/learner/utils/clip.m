function  y = clip( x, lo, hi )
 % ============> FAST VERSION <=============
 %  (uses matrix Logicals to replace Loops)
 y = (x .* [x<=hi])  +  (hi .* [x>hi]);
 y = (y .* [x>=lo])  +  (lo .* [x<lo]);
