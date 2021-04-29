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

## start
| Column | Description |
|---|---|
1 | s
2 | starting node index


## goal
| Column | Description |
|---|---|
1 | g
2 | goal node index
