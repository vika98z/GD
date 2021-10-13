using UnityEngine;

public class CheckCollision : MonoBehaviour
{
  public bool IsBlocked;

  private const string Tag = "RoboPart";

  private void OnTriggerEnter(Collider other)
  {
    if (other.CompareTag(Tag))
      IsBlocked = true;
  }

  private void OnTriggerExit(Collider other)
  {
    if (other.CompareTag(Tag))
      IsBlocked = false;
  }
}