# Property.msg
# Represents a single BIM property/parameter (key-value pair with type metadata)

string key          # Parameter name (e.g., "Height", "Fire Rating")
string value        # Parameter value as string
string type         # Data type hint (e.g., "Length", "Text", "Boolean", "Number")
