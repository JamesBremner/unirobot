Note:  This is no longer supported.  It has been replaced by a pecialization of a general purpose path finder engine.  See https://github.com/JamesBremner/PathFinder

# unirobot
Find path for unidirectional robot in bidirectional graph

For motivation, see https://stackoverflow.com/questions/67316885/filter-a-graph-by-dynamic-conditions-constraints

Input is a space delimited text file describing the links, stadting position and goal

## Links
| Column | Description |
|---|---|
1 | l for link
2 | src node index
3 | dst node index
4 | link description :<br> 0, any orientation <br> 1, forwards only<br>2, backwards only
5 | cost

## turning nodes
| Column | Description |
|---|---|
1 | t for turning node
2 | index of node where robot can turn around
3 | not used, enter "."
4 | not used, enter "."
5 | cost

## start
| Column | Description |
|---|---|
1 | s
2 | starting node index
3 | starting orientation:<br> f, forward <br> b, backward 


## goal
| Column | Description |
|---|---|
1 | g
2 | goal node index

## Example input

<img src="https://i.stack.imgur.com/o9uiq.png" ></a>

```
l 1 2 0 1
l 2 3 2 1
l 2 4 1 1
l 4 3 1 1
t 3 . . 0
s 1 f
g 3
```

## Example Output

Sample output when robot starts at 1 facing forwards and heading for 4 ( the robot can get there directly via 2 )

```
C:\Users\James\code\unirobot\bin>unirobot.exe inforward.txt
unirobot
l 1 2 0 1
l 2 3 2 1
l 2 4 1 1
l 4 3 1 1
t 3 . . 0
s 1 f
g 4
4 bidirectional links input

Forward links
1f - 2f
2f - 4f
4f - 3f

Back links
1b - 2b
2b - 3b

Turning Links
3f - 3b

Combined graph links
(1f,2f) (2f,1f) (2f,4f) (4f,2f) (4f,3f) (3f,4f) (3f,3b) (1b,2b) (2b,1b) (2b,3b) (3b,2b) (3b,3f)

Path: 1f -> 2f -> 4f ->
```

Sample output when starts at 1 facing backwards and heading for 4 ( The robot needs to go via node 3, the only place it can turn around )

```
C:\Users\James\code\unirobot\bin>unirobot.exe inforward.txt
unirobot
l 1 2 0
l 2 3 2
l 2 4 1
l 4 3 1
t 3
s 1 f
g 4
4 bidirectional links input

Forward links
1f - 2f
2f - 4f
4f - 3f

Back links
1b - 2b
2b - 3b

Turning Links
3f - 3b

Combined graph links
(1f,2f) (2f,1f) (2f,4f) (4f,2f) (4f,3f) (3f,4f) (3f,3b) (1b,2b) (2b,1b) (2b,3b) (3b,2b) (3b,3f)

Path: 1f -> 2f -> 4f ->

```

