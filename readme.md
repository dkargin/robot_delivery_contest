

/// Estimated size of a single from = ~16 bytes.
/// Priority queue will need another 4 byte per from (very worst case).
/// Path ID or visitTime will take 4 bytes.
/// Max size of a map Nmax = 2000
/// Grid size = sizeof(Node) + sizeof(NodeID) + 4 = 24 * 2000 * 2000 ~ 96Mb.
/// 


Input
```
4 20 10
....
....
....
....
7 7
1
1 1 4 4  (1)
1
1 4 4 1  (2)
1
4 4 1 1  (3)
0
4
1 2 4 4  (4)
2 2 3 3  (5) - Problem here
2 1 4 4  (6)
2 2 4 4  (7)
0
0
```
wrong answer Rover 1 can't finish order 5 at iteration 6 because he does not reach destination yet


Interactor exit code : 1