#define UNITY_INDIRECT_DRAW_ARGS IndirectDrawArgs

//#include "UnityIndirect.cginc"

StructuredBuffer<float3> Vertices;
StructuredBuffer<int> Triangles;
StructuredBuffer<float3> Normals;
StructuredBuffer<float2> UVs;
StructuredBuffer<int> TriangleOffsets;
StructuredBuffer<uint> TriangleLocalStartIndex;
StructuredBuffer<uint> TextureDoubleSided;

uint numSides;
uint oneSidedNumTriangles;
//uint offset;

void GetVertexData_float(uint vertexID : SV_VertexID, out float3 position, out float3 normal, out float2 texcoord, out float2 texcoord2)
{
    //InitIndirectDrawArgs(0);
    
    uint sideID = 0;
    
    if (numSides > 0)
    {
        for (uint i = 1; i < numSides; i++)
        {
            if (vertexID < TriangleLocalStartIndex[i])
            {
                sideID = i - 1;
                break;
            }
            else if (i == numSides - 1)
            {
                sideID = i;
                break;
            }

        }
    }
    
    uint offsetID = vertexID + TriangleOffsets[sideID] - TriangleLocalStartIndex[sideID];
    
    uint index = Triangles[offsetID];
    
    //position = Vertices[Triangles[GetIndirectVertexID(vertexID)] + 0];
    float3 localPos = Vertices[index];
    
    //position = mul(InstanceTransforms[instanceID], float4(localPos, 1.0f)).xyz;
    position = localPos;
    
    //normal = mul((float3x3) InstanceTransforms[instanceID], Normals[index] * (offsetID >= oneSidedNumTriangles ? -1 : 1)).xyz;
    
    if (offsetID >= oneSidedNumTriangles)
    {
        normal = Normals[index] * -1;
        texcoord2 = TextureDoubleSided[sideID] == 1 ? UVs[index] : float2(0.0f, 0.0f);
    }
    else
    {
        normal = Normals[index];
        texcoord2 = UVs[index];
    }
    
    texcoord = UVs[index];
}