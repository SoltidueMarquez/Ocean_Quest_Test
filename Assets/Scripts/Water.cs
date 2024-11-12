using UnityEngine;
using Random = UnityEngine.Random;

namespace DefaultNamespace
{
    public class Water : MonoBehaviour
    {
        public static Water Instance;

        private void Awake()
        {
            Instance = this;
        }

        public float GetWaterHeight(Vector3 vector3)
        {
            return Random.Range(-0.5f, 0.5f);
        }
    }
}