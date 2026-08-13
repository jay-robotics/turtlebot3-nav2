import os
import cv2

script_dir = os.path.dirname(os.path.abspath(__file__))

image1_path = os.path.join(script_dir, "book_view_1.png")
image2_path = os.path.join(script_dir, "book_view_2.png")

image1 = cv2.imread(image1_path)
image2 = cv2.imread(image2_path)

if image1 is None:
    raise FileNotFoundError(f"Could not read image: {image1_path}")
if image2 is None:
    raise FileNotFoundError(f"Could not read image: {image2_path}")

print("Image 1:", image1.shape)
print("Image 2:", image2.shape)

orb = cv2.ORB_create()

keypoints1, descriptors1 = orb.detectAndCompute(image1, None)
keypoints2, descriptors2 = orb.detectAndCompute(image2, None)

matcher=cv2.BFMatcher(cv2.NORM_HAMMING)

matches=matcher.knnMatch(
    descriptors1,
    descriptors2,
    k=2
)

good_matches=[]

for pair in matches:

    if len(pair)<2:
        continue

    best=pair[0]
    second=pair[1]

    if best.distance<0.75*second.distance:
        good_matches.append(best)

for match in good_matches[:10]:

    kp1 = keypoints1[match.queryIdx]
    kp2 = keypoints2[match.trainIdx]

    x1, y1 = kp1.pt
    x2, y2 = kp2.pt

    print(
        f"Image 1: ({x1:.1f}, {y1:.1f}) "
        f"-> Image 2: ({x2:.1f}, {y2:.1f}) "
        f"distance={match.distance:.1f}"
    )

    
match_image=cv2.drawMatches(
    image1,
    keypoints1,
    image2,
    keypoints2,
    good_matches,
    None,
    flags=cv2.DrawMatchesFlags_NOT_DRAW_SINGLE_POINTS
)

cv2.imshow("book feature matches",match_image)
cv2.waitKey(0)
cv2.destroyAllWindows()



print("raw parts",len(matches))
print("good matches",len(good_matches))
print("Image 1 keypoints:", len(keypoints1))
print("Image 2 keypoints:", len(keypoints2))

print("Image 1 descriptors:", descriptors1.shape)
print("Image 2 descriptors:", descriptors2.shape)