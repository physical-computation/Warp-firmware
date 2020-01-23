1.	Code indented with tabs, not spaces. For visual alignment, assume the code will be viewed in an editor where tabs are rendered to be as wide as eight spaces.

2.	Variable names in `camelCase`.

3.	Type names begin with a capital. Type names specific to Warp begin with `Warp`, e.g., `WarpPhenomenon`.

4.	Constant names and enum entries begin with `kWarp`, e.g., `kWarpPhenomenonTemperatureCelcius`. 

5.	C-style comments, in the form:
````c
	/*
	 *	Comment (offset with a single tab)
	 */
````

6.	No `#include` within header .h files if possible.

7.	No function definitions in header .h files.

8.	Files named named `warp-xxxcamelCasedName.yyy`.

9.	Constants in `enum`s, not in `#define`s where possible.

10.	Avoid `#define` if possible.

11.	All `if` statement followed by curly braces, even if body is a single statement.
