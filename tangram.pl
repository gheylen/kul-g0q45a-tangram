% Tangram in SWI-Prolog Copyright (c) 2011 Glenn Heylen
% 
% This file is part of Tangram in SWI-Prolog.
% 
% Tangram in SWI-Prolog is free software: you can redistribute it and/or modify
% it under the terms of the GNU General Public License as published by
% the Free Software Foundation, either version 3 of the License, or
% (at your option) any later version.
% 
% Tangram in SWI-Prolog is distributed in the hope that it will be useful,
% but WITHOUT ANY WARRANTY; without even the implied warranty of
% MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
% GNU General Public License for more details.
% 
% You should have received a copy of the GNU General Public License
% along with Tangram in SWI-Prolog.  If not, see <http://www.gnu.org/licenses/>.

% Solution to practicum of course G0Q45a @ Katholieke Universiteit Leuven, Belgium.

:- use_module(library(clpfd)).

% All possible convex puzzles (Pass one of these to the tangram predicate)
% [(0,0),(4,0),(4,4),(0,4)]
% [(3,0),(5,2),(5,4),(4,5),(2,5),(0,3)]
% [(0,0),(6,0),(7,1),(7,3),(3,3)]
% [(0,1),(1,0),(5,0),(7,2),(6,3),(2,3)]
% [(0,3),(3,0),(7,0),(2,5)]
% [(0,0),(4,0),(4,6),(0,2)]
% [(0,0),(8,0),(4,4)]
% [(2,0),(4,0),(6,2),(4,4),(2,4),(0,2)]
% [(1,0),(5,4),(4,5),(0,5),(0,1)]
% [(0,0),(2,0),(5,3),(5,5),(3,5),(0,2)]
% [(0,0),(4,0),(6,2),(6,6)]
% [(4,0),(8,0),(4,4),(0,4)]
% [(2,0),(6,4),(4,6),(0,2)]

% Area domain
width(8).
height(6).

% Blocks
block(1,[(0,0),(4,0),(2,2)]). % Large triangle 
block(2,[(0,0),(4,0),(2,2)]). % Large triangle 
block(3,[(0,0),(2,0),(0,2)]). % Medium triangle
block(4,[(0,0),(2,0),(1,1)]). % Small triangle 
block(5,[(0,0),(2,0),(1,1)]). % Small triangle 
block(6,[(0,0),(1,-1),(2,0),(1,1)]). % Square 
block(7,[(0,0),(1,1),(1,3),(0,2)]). % Parallelogram

tangram(Puzzle, FormattedPuts) :-
	length(Puts, 21),
	availableAreaAndDomain(Area),
	areaConstraintsForPuzzle(Area, Puzzle),
	blockPutConstraintsAsTuples(Tuples, Puzzle),
	blockPutConstraintsByTuples(Puts, Tuples),
	symmetryConstraints(Puts),
	blockPutConstraints(Tuples, Puts, Area),
	labeling([ff], Puts),
	formatPuts(Puts, FormattedPuts).

% Expanding factor. E.g.: How many triangle per part of area of size 1x1.
growingFactor(4).

% Rounding flexibility.
epsilon(0.01).

% Rotation limitations. Multiplied by 45. 
% E.g.: 6 * 45 = 270. Max angle tested will be 270^o.
% Tests use stepvalue 2. So tests will include angles: 0, 90, 180, 270.
maxAngle(6).

% How big is our area used for calculations?
% Each part of the area represents a triangle. (Non-game piece)
% The domain of the value of each triangle is constrained to 0..7.
availableAreaAndDomain(Area) :-
	amountOfTriangles(AmountOfTriangles),
	length(Area, AmountOfTriangles),
	Area ins 0..7.

% Self-explanatory
amountOfTriangles(AmountOfTriangles) :-
	width(Width), 
	height(Height),
	growingFactor(Grow),
	AmountOfTriangles is Width * Height * Grow.

% Entry call. Delegates and start loop of putting up constraints for each triangle.	
areaConstraintsForPuzzle(Area, Puzzle) :-
	areaConstraintsForPuzzle(0, Area, Puzzle).

% Determines for each triangle whether or not it's a part of the current puzzle-to-be-solved.
% If it's a part -> value of current triangle iterated should be bigger than 0.
% If not -> value of current triangle iterated is 0.
areaConstraintsForPuzzle(_, [], _).
areaConstraintsForPuzzle(I, [A|Rea], Puzzle) :-
	(
		\+ triangleInPuzzle(I, Puzzle) -> A = 0;
		true  -> A #> 0
	),
	I1 is I+1,
	areaConstraintsForPuzzle(I1, Rea, Puzzle).

% Determines whether or not a triangle is part of the current puzzle-to-be-solved.
triangleInPuzzle(T, Puzzle) :-
	triangleVerticesOfSquareArea(T, X1, Y1, X2, Y2, X3, Y3),
	pointInFigure(X1, Y1, Puzzle),
	pointInFigure(X2, Y2, Puzzle),
	pointInFigure(X3, Y3, Puzzle).

% Determines the vertices of a specific triangle.
triangleVerticesOfSquareArea(T, X1, Y1, X2, Y2, X3, Y3) :-
	width(Width),
	growingFactor(Grow),
	ExpandedWidth is (Width * Grow),
	I is (T mod ExpandedWidth) // Grow,
	J is T // ExpandedWidth,
	P is T mod Grow,
	(
		P = 0 -> ( X1 = I, Y1 = J, X2 = I, Y2 is J + 1);
		P = 1 -> ( X1 = I, Y1 = J, X2 is I + 1, Y2 = J);
		P = 2 -> ( X1 is I + 1, Y1 = J, X2 is I + 1, Y2 is J + 1);
		P = 3 -> ( X1 = I, Y1 is J + 1, X2 is I + 1, Y2 is J + 1)
	),
	X3 is I + 0.5, Y3 is J + 0.5.

% Determines whether or not P = (X, Y) is in a figure.
% Figure expects a list of coordinates. E.g.: [(0, 1), (1, 2), (2, 3)]
pointInFigure(X, Y, Figure) :-
	member((X, Y), Figure).
pointInFigure(X, Y, Figure) :-
	\+ member((X, Y), Figure),
	biVertexOfFigure(X, Figure, Y1, Y2),
	Y1 < Y2,
	Y1 =< Y, Y =< Y2.
pointInFigure(X, Y, Figure) :-
	\+ member((X, Y), Figure),
	\+ biVertexOfFigure(X, Figure, Y1, Y2),
	vertexOfFigure(X, Figure, Yv),
	figureEdge(X, (X1,Y1), (X2,Y2), Figure),
	X \= X1, X \= X2,
	(
		Y > Yv, (X - X1) * (Y2 - Y1) >= (Y - Y1) * (X2 - X1); 
		Y < Yv, (X - X1) * (Y2 - Y1) =<  (Y - Y1) * (X2 - X1)
	).
pointInFigure(X, Y, Figure) :-
	\+ member((X, Y), Figure),
	\+ vertexOfFigure(X, Figure, _),	% not(biVertexOfFigure) is covered at this line as well.
	figureEdge(X, (X1, Y1), (X2, Y2), Figure), 
	figureEdge(X, (X3, Y3), (X4, Y4), Figure),
	(Y1 < Y3, !; Y1 = Y3, Y2 < Y4),
	(X - X1) * (Y2 - Y1) =< (Y - Y1) * (X2 - X1), 
	(X - X3) * (Y4 - Y3) >= (Y - Y3) * (X4 - X3).

% Determines a Y coordinate value for a given X. Hence, finding out whether or not
%	X is a position where two edges of a figure meet.
% What is the Y value of a coordinate of a figure where the X value is given?
vertexOfFigure(X, Figure, Y1) :-
	member((X, Y1), Figure).
	
% Determines two Y coordinate values for a given X. Hence, finding out whether or not
%	X is a position where two times two edges of a figure meet.
% What are two distinct Y values of two coordinates of a figure where the X value is given?
biVertexOfFigure(X, Figure, Y1, Y2) :-
	member((X, Y1), Figure),
	member((X, Y2), Figure),
	Y1 \= Y2.

% Determines two coordinates (and essentially an edge; line), which relate to a current X position.
% Note: This could be cleaned up a little further in seperate predicates to improve readability.
figureEdge(X, (Xa, Ya), (Xb, Yb), Figure) :-
	length(Figure, N),
	member((Xa,Yaa), Figure),
	member((Xb,Ybb), Figure),
	Xa =< X, X  =< Xb,
	(
		Xa = Xb ->
		(
			Ya = min(Yaa, Ybb),
			Yb = max(Yaa, Ybb)
		);
		true ->
		(
			Ya = Yaa,
			Yb = Ybb
		)
	),
	SubMax is N - 1,
	nth0(Line1, Figure, (Xb, Ybb)),
	nth0(Line2, Figure, (Xa, Yaa)),
	(
		Line1 is Line2 + 1;
		Line2 is Line1 + 1;
		nth0(0, Figure,(Xa, Yaa)), nth0(SubMax, Figure, (Xb, Ybb));
		nth0(SubMax, Figure, (Xa, Yaa)), nth0(0, Figure, (Xb, Ybb))
	).

% Will look for a list of lists of a combination of coordinates and an angle.
% E.g.: [[(0, 1, 6), (1, 5, 4)], [(0, 4, 0), (1, 2, 4), (4, 6, 2)]]
% The list will contain all the possibile placements (puts) for each puzzle block in the puzzle area.
blockPutConstraintsAsTuples(Tuples, Figure) :-
	blockPutConstraintsAsTuples(1, Tuples, Figure).

blockPutConstraintsAsTuples(8, [], _).
blockPutConstraintsAsTuples(Block, [T|Uples], Figure) :-
    width(Width),
	height(Height),
    blockPutConstraintsAsTuples(0, Width, 0, Height, 0, Block, T, Figure),
    NextBlock is Block + 1,
    blockPutConstraintsAsTuples(NextBlock, Uples, Figure).

blockPutConstraintsAsTuples(X, Width, _, _, _, _, [], _) :- 
	X > Width.
blockPutConstraintsAsTuples(X, Width, Y, Height, _, Block, Tuple, Figure) :-
	X =< Width,
	Y > Height,
	X1 is X + 1,
	blockPutConstraintsAsTuples(X1, Width, 0, Height, 0, Block, Tuple, Figure).
blockPutConstraintsAsTuples(X, Width, Y, Height, A, Block, Tuple, Figure) :-
	X =< Width,
	Y =< Height,
	maxAngle(MaxAngle),
	A > MaxAngle,
	Y1 is Y + 1,
	blockPutConstraintsAsTuples(X, Width, Y1, Height, 0, Block, Tuple, Figure).
blockPutConstraintsAsTuples(X, Width, Y, Height, A, Block, [[X, Y, A]|Uple], Figure) :-
	X =< Width,
	Y =< Height,
	maxAngle(MaxAngle),
	A =< MaxAngle,
	block(Block, BlockCoords),
	blockInFigure(X, Y, A, BlockCoords, Figure),
	A1 is A + 2,
    blockPutConstraintsAsTuples(X, Width, Y, Height, A1, Block, Uple, Figure).
blockPutConstraintsAsTuples(X, Width, Y, Height, A, Block, Tuple, Figure) :-
	X =< Width,
	Y =< Height,
	maxAngle(MaxAngle),
	A =< MaxAngle,
	A1 is A + 2,
	% Commented for performance optimization.
	% This predicate should remain beneath all the others, unless lines underneath are uncommented.
	% block(Block, BlockCoords),
	% \+ blockInFigure(X, Y, A, BlockCoords, Figure),
	blockPutConstraintsAsTuples(X, Width, Y, Height, A1, Block, Tuple, Figure).

% Determines whether or not a given block is inside the figure (puzzle).
% Will shift and rotate each edge of the block to the current iterated position and angle. (X, Y, A)
blockInFigure(_, _, _, [], _).
blockInFigure(X, Y, Angle, [(Xi, Yi)|Block], Figure) :-
	lineRotationAndShift(X, Y, Xi, Yi, Angle, X1n, Y1n),
	pointInFigureEpsilon(X1n, Y1n, Figure),
	blockInFigure(X, Y, Angle, Block, Figure).

% Determines the line by rotating and shifting it.
% It accepts (0) the values of the current iterated position and
%	(1) the vector building an edge of a block, which will be rotated and shifted.
% The magic number 0.7853.. is 45^o in radians.
lineRotationAndShift(X0, Y0, X1, Y1, Angle, RotatedX1, RotatedY1) :-
    Cos is cos(Angle * 0.7853981633974484),
    Sin is sin(Angle * 0.7853981633974484),
    RotatedX1  is X1 * Cos - Y1 * Sin + X0,
    RotatedY1  is Y1 * Cos + X1 * Sin + Y0.
	
% Idem explanation pointInFigure.
% Only difference: will convert the given coordinates to ints, if they're pretty much an integer.
pointInFigureEpsilon(X, Y, Figure) :-
	closeToInteger(X, Y, Xint, Yint),
	pointInFigure(Xint, Yint, Figure).	

% Checks whether a coordinate is pretty much an integer.
closeToInteger(X, Y, X1, Y1) :-
	epsilon(Eps),
	X1 is round(X),
	Y1 is round(Y),
	abs(X - X1) + abs(Y - Y1) < Eps.

% Uses a list of lists of tuples in format [X, Y, A].
% E.g.: [[[0, 1, 6], [1, 5, 4]], [[0, 4, 0], [1, 2, 4], [4, 6, 2)]]
% Puts constraints on each trio of variables in Puts.
% Each put should be a legal put as determined before by the predicate blockPutConstraintsAsTuples.
% We now have limited the results to puts of the puzzle blocks inside the puzzle area.
% Overlapping puzzle blocks aren't constraint by this predicate.
blockPutConstraintsByTuples([], []).
blockPutConstraintsByTuples([X, Y, A|Uts], [T|Uples]) :-
	tuples_in([[X, Y, A]], T),
	blockPutConstraintsByTuples(Uts, Uples).

% Adds some optimization constraints to the final put of each puzzle block.
% Each block has its own limitations.
symmetryConstraints([X1, Y1, _, X2, Y2, _, _, _, _, X4, Y4, _, X5, Y5, _, _, _, A6, _, _, A7]) :-
	X1 #=< X2,
	X1 #= X2 #==> Y1 #< Y2, 	% Both large triangles
	X4 #=< X5,
	X4 #= X5 #==> Y4 #< Y5, 	% Both small triangles
	A6 #= 0,               		% Square is always correctly rotated in our convex puzzles.
	A7 #< 4. 					% Parallelogram

% Determines constraints about puzzle blocks in relation to each other.
% No block should overlay another block.
% If we put a block at a certain coordinate using a certain angle, all of the triangles covering
%	that area of the figure (puzzle) should have the same value.
% Constraints concerning possible values for the triangles and whether or not a triangle makes up
%	a puzzle, aren't covered here.
% If we're choosing certain values from a tuple as final put value, all of triangles which make up a
%	particular block at that put should have the same value.
%	Thus, Not covered by another value(Read: block; each block has an unique id).
blockPutConstraints(Tuples, Puts, Area) :-
    blockPutConstraints(Tuples, Puts, Area, 1).

blockPutConstraints([], _, _, _).
blockPutConstraints([T|Uples], [X, Y, A|Uts], Area, Block) :-
    blockPutConstraints(T, X, Y, A, Area, Block),
    NextBlock is Block + 1,
    blockPutConstraints(Uples, Uts, Area, NextBlock).

blockPutConstraints([], _, _, _, _, _).
blockPutConstraints([[X, Y, A]|Uple], Xo, Yo, Ao, Area, Block) :-
	amountOfTriangles(AmountOfTriangles),
    findall(
		T,
		(
			between(0, AmountOfTriangles, T),	% More or less counter: for(int T = 0; T < N; T++);
			triangleInBlock(Block, X, Y, A, T)
		),
		TrianglesInBlock
	),
	sort(TrianglesInBlock, TrianglesInBlockSorted),	% Performance. Also removes duplicates from list.
    bulkNth0(TrianglesInBlockSorted, Area, TheseTriangles),	% Get all effective triangle values.
    valuesInListAllEqualTo(TheseTriangles, Block, C),	% All triangle values should be the same.
    (Xo #= X #/\ Yo #= Y #/\ Ao #= A) #==> C,	% Limit puts last time.
    blockPutConstraints(Uple, Xo, Yo, Ao, Area, Block).

% Determines whether or not a specified triangle is inside a specified block.
triangleInBlock(Block, X, Y, Angle, Triangle) :-
	triangleVerticesOfSquareArea(Triangle, X1, Y1, X2, Y2, X3, Y3),
	triangleEdgeInBlock(Block, X, Y, Angle, X1, Y1),
	triangleEdgeInBlock(Block, X, Y, Angle, X2, Y2),
	triangleEdgeInBlock(Block, X, Y, Angle, X3, Y3).

% Determines whether or not a specified edge of a triangle is in a specified block.
triangleEdgeInBlock(B, X0, Y0, Angle, X4, Y4) :-
	block(B, BlockCoordinates),
	length(BlockCoordinates, Edges),
	Edges = 3,
	block(B, [_, (X1, Y1), (X2, Y2)]),
	lineRotationAndShift(X0, Y0, X1, Y1, Angle, X1n, Y1n),
	lineRotationAndShift(X0, Y0, X2, Y2, Angle, X2n, Y2n),
	triangleEdgeInBlock(X4, Y4, X0, Y0, X1n, Y1n, X2n, Y2n).

triangleEdgeInBlock(B, X0, Y0, Angle, X4, Y4) :-
	block(B, BlockCoordinates),
	length(BlockCoordinates, Edges),
	Edges = 4,
	block(B, [(_, _), (X1, Y1), (X2, Y2), (X3, Y3)]),
	lineRotationAndShift(X0, Y0, X1, Y1, Angle, X1n, Y1n),
	lineRotationAndShift(X0, Y0, X2, Y2, Angle, X2n, Y2n),
	lineRotationAndShift(X0, Y0, X3, Y3, Angle, X3n, Y3n),
	triangleEdgeInBlock(X4, Y4, X0, Y0, X1n, Y1n, X2n, Y2n, X3n, Y3n).

triangleEdgeInBlock(X4, Y4, X0, Y0, X1n, Y1n, X2n, Y2n, _, _) :-
     triangleEdgeInBlock(X4, Y4, X0, Y0, X1n, Y1n, X2n, Y2n).
triangleEdgeInBlock(X4, Y4, X0, Y0, _, _, X2n, Y2n, X3n, Y3n) :-
     triangleEdgeInBlock(X4, Y4, X0, Y0, X2n, Y2n, X3n, Y3n).
triangleEdgeInBlock(X, Y, X0, Y0, X1, Y1, X2, Y2):-
    triangleEdgeInVertexOfBlock(X, Y, X0, Y0, X1, Y1, X2, Y2),
    triangleEdgeInVertexOfBlock(X, Y, X1, Y1, X2, Y2, X0, Y0),
    triangleEdgeInVertexOfBlock(X, Y, X2, Y2, X0, Y0, X1, Y1).

% Using vectors to determine whether or not an vertex is on the same side.
triangleEdgeInVertexOfBlock(X, Y, VectorToEvalX, VectorToEvalY, Vector1X, Vector1Y, Vector2X, Vector2Y) :-
	X0 is Vector2X - Vector1X,
	Y0 is Vector2Y - Vector1Y,
	X1 is VectorToEvalX - Vector1X,
	Y1 is VectorToEvalY - Vector1Y,
	X2 is X - Vector1X,
	Y2 is Y - Vector1Y,
	CZ is X0 * Y1 - X1 * Y0,
	DZ is X0 * Y2 - X2 * Y0,
	epsilon(Eps),
	CZ * DZ >= -Eps.

% Self-explanatory
bulkNth0([], _, []).
bulkNth0([I|Ndex], List, [V|Alues]) :-
	nth0(I, List, V),
	bulkNth0(Ndex, List, Alues).

% Determines whether or not all values in a specified list are equal to Value.
valuesInListAllEqualTo([], _, 1).
valuesInListAllEqualTo([L|Ist], Value, C) :-
	valuesInListAllEqualTo(Ist, Value, C1),
	C #<==> C1 #/\ L #= Value.
	
% Formats the list Puts in a requested format.
% Usage of less noisy version of the list Puts to improve performance and readability.
%
% Intern usage of list description:
%	No seperation per move or put ([0, 1, 0, 2, 1, 4] instead of [(0, 1, 0), (2, 1, 4)])
%		This also makes final labeling easier;
%	Angles always used in calculations in multiples of 45^o in radians.
%
% Edit MyFormat to change returning list's format.
formatPuts(Puts, FormattedPuts) :-
	formatPuts(Puts, FormattedPuts, 1).
	
formatPuts([], [], _).
formatPuts([X, Y, A|Puts], [MyFormat|FormattedPuts], Block) :-
	AngleInDegrees is A * 45,
	MyFormat = put(Block, ((X, Y), AngleInDegrees)),
	NewBlock is Block + 1,
	formatPuts(Puts, FormattedPuts, NewBlock).