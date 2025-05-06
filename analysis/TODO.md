## Refactor:

1. Remove the giant DF in `warehouse.py`. Create a dict with names and arrays and use that in a `create_df()` method to individually create DFs.
2. Filter each df in a similar manner: pass each df into a function with the transition events. Return the filtered df