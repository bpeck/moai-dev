kdTree = MOAIKDTree3fi.new()

pts = {
-- x,y,z,a   (a is integer data value)
   1,2,3,4,
   5,6,7,8,
   5,6,7,4,
   3,4,5,6,
   7,8,5,4,
   5,4,3,1,
}

kdTree:setData(pts)

N = 2
res = kdTree:KNN(N,0,2,3)

print("KNN results:")
for i, triplet in pairs(res) do
    cloud_idx, dist, data_value = unpack(triplet)
    print("[" .. i .. "]: idx=" .. cloud_idx .. ", dist=" .. dist .. ", data=" .. data_value)
end