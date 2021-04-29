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

## turning nodes
| Column | Description |
|---|---|
1 | t for turning node
2 | index of node where robot can turn around

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
l 1 2 0
l 2 3 2
l 2 4 1
l 4 3 1
t 3
s 1 f
g 3
```
