from rapidfuzz import process

# Sample data
choices = ["apple", "banana", "grape", "orange", "pineapple", "blueberry"]

# Find best matches for the query
query = "app"
best_matches = process.extract(query, choices, limit=3)
print(f"Best matches for '{query}': {best_matches}")