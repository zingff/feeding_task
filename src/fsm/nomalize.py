import numpy as np

def normalize_quaternion(q):
    # Calculate magnitude
    magnitude = np.linalg.norm(q)
    
    # Normalize quaternion
    normalized_q = q / magnitude
    
    return normalized_q

# Original quaternion
q = np.array([-0.315519, 0.508754, 0.696705, -0.395247])

# Normalize the quaternion
normalized_q = normalize_quaternion(q)

print("Original Quaternion:", q)
print("Normalized Quaternion:", normalized_q)
