using System.Collections.Generic;
using DefaultNamespace;
using UnityEngine;

/// <summary>
/// 浮力脚本，将流体分割为若干个voxel独立计算
/// </summary>
public class Buoyancy : MonoBehaviour
{
    [SerializeField, Tooltip("沿着坐标轴分割的次数")] private int slicesPerAxis = 2;
    [SerializeField, Tooltip("分割出来的voxel总数限制")] private int voxelsLimit = 16;
    [SerializeField, Tooltip("密度")] private float density = 500f;

    [Tooltip("水的密度常量")] private const float WATER_DENSITY = 1000f;
    [Tooltip("阻力系数")] private const float DAMPFER = 0.1f;

    [Tooltip("voxel高度计算")] private float VoxelHalfHeight { get; set; }
    [Tooltip("单个voxel受到的浮力大小")] private Vector3 LocalArchimedesForce { get; set; }
    [Tooltip("存放voxel的容器")] private List<Vector3> Voxels { get; set; }
    private bool IsMeshCollider { get; set; }
    [Tooltip("用于可视化的力容器")] private List<Vector3[]> Forces { get; set; }

    private Rigidbody Rb { get; set; }
    private Collider Col { get; set; }

    private void Start()
    {
        InitializeComponents();
        CalculateVoxelHalfHeight();
        SetupRigidbody();
        CreateVoxels();
        CalculateArchimedesForce();
        Forces = new List<Vector3[]>();
    }

    private void FixedUpdate()
    {
        Forces.Clear();
        //遍历所有的voxel计算浮力与阻力
        foreach (var point in Voxels)
        {
            ApplyBuoyancyForce(point);
        }
    }

    private void InitializeComponents()
    {
        Col = GetComponent<Collider>() ?? gameObject.AddComponent<MeshCollider>();
        IsMeshCollider = Col is MeshCollider;
        Rb = GetComponent<Rigidbody>() ?? gameObject.AddComponent<Rigidbody>();
    }

    
    private void SetupRigidbody()
    {
        var bounds = Col.bounds;
        Rb.centerOfMass = transform.InverseTransformPoint(bounds.center);
    }


    #region voxel相关
    private void CreateVoxels()
    {
        Voxels = SliceConvex();
        WeldPoints(Voxels, voxelsLimit);
    }
    
    private void CalculateVoxelHalfHeight()
    {
        var bounds = Col.bounds;
        VoxelHalfHeight = Mathf.Min(bounds.size.x, bounds.size.y, bounds.size.z) / (2 * slicesPerAxis);
    }

    //根据包围盒和三个坐标轴的分割序号来计算对应voxel中心的位置
    private Vector3 CalculateVoxelPoint(Bounds bounds, int ix, int iy, int iz)
    {
        var x = bounds.min.x + bounds.size.x / slicesPerAxis * (0.5f + ix);
        var y = bounds.min.y + bounds.size.y / slicesPerAxis * (0.5f + iy);
        var z = bounds.min.z + bounds.size.z / slicesPerAxis * (0.5f + iz);
        return transform.InverseTransformPoint(new Vector3(x, y, z));//世界坐标转局部坐标
    }

    //对三个轴进行分割
    private List<Vector3> SliceConvex()
    {
        var points = new List<Vector3>();
        var bounds = Col.bounds;
        for (int ix = 0; ix < slicesPerAxis; ix++)
        {
            for (int iy = 0; iy < slicesPerAxis; iy++)
            {
                for (int iz = 0; iz < slicesPerAxis; iz++)
                {
                    points.Add(CalculateVoxelPoint(bounds, ix, iy, iz));
                }
            }
        }
        return points;
    }

    //根据最大容量属性对分割好的voxel处理,如超出限制则不断找最近的两个voxel合并
    private static void WeldPoints(IList<Vector3> list, int targetCount)
    {
        if (list.Count <= 2 || targetCount < 2)//数量过少直接返回
        {
            return;
        }

        while (list.Count > targetCount)
        {
            int first, second;
            FindClosestPoints(list, out first, out second);

            var mixed = (list[first] + list[second] * 0.5f);
            list.RemoveAt(second);
            list.RemoveAt(first);
            list.Add(mixed);
        }
    }
    
    //功能函数，找到所有voxcel中最近的两个
    private static void FindClosestPoints(IList<Vector3> list, out int firstIndex, out int secondIndex)
    {
        float minDistance = float.MaxValue;
        firstIndex = 0;
        secondIndex = 1;

        for (int i = 0; i < list.Count; i++)
        {
            for (int j = i + 1; j < list.Count; j++)
            {
                float distance = Vector3.Distance(list[i], list[j]);
                if (distance < minDistance)
                {
                    minDistance = distance;
                    firstIndex = i;
                    secondIndex = j;
                }
            }
        }
    }
    #endregion

    #region 浮力计算
    //计算浮力
    private void CalculateArchimedesForce()
    {
        float volume = Rb.mass / density; //计算总体积
        float archimedesForceMagnitude = WATER_DENSITY * Mathf.Abs(Physics.gravity.y) * volume; //计算总浮力
        LocalArchimedesForce = new Vector3(0, archimedesForceMagnitude, 0) / Voxels.Count;
    }
    
    //获取水面高度函数
    private float GetWaterLevel(float x, float z)
    {
        if (Water.Instance != null)
            return Water.Instance.GetWaterHeight(new Vector3(x, 0, z));

        return 0;
    }

    //计算单个voxel的受浮力情况
    private void ApplyBuoyancyForce(Vector3 point)
    {
        var worldPoint = transform.TransformPoint(point);//局部坐标转换世界坐标
        float waterLevel = GetWaterLevel(worldPoint.x, worldPoint.z);

        if (worldPoint.y - VoxelHalfHeight < waterLevel)//如果在水面以下
        {
            float k = Mathf.Clamp01((waterLevel - worldPoint.y) / (2 * VoxelHalfHeight) + 0.5f);//计算水面以下比值
            var velocity = Rb.GetPointVelocity(worldPoint);
            var localDampingForce = -velocity * (DAMPFER * Rb.mass);
            var force = localDampingForce + k * LocalArchimedesForce;//应用浮力阻力
            Rb.AddForceAtPosition(force, worldPoint);
            
            Forces.Add(new[] { worldPoint, force });
        }
    }
    #endregion

    #region 浮力可视化
    private void OnDrawGizmos()
    {
        if (Voxels == null || Forces == null)
        {
            return;
        }
        
        const float gizmoSize = 0.05f;
        Gizmos.color = Color.yellow;
        foreach (var p in Voxels)
        {
            Gizmos.DrawCube(transform.TransformPoint(p), new Vector3(gizmoSize, gizmoSize, gizmoSize));
        }

        Gizmos.color = Color.cyan;
        foreach (var force in Forces)
        {
            Gizmos.DrawCube(force[0], new Vector3(gizmoSize, gizmoSize, gizmoSize));
            Gizmos.DrawLine(force[0], force[0] + force[1] / Rb.mass);
        }
    }
    #endregion
}
