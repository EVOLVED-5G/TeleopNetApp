resource "kubernetes_pod" "TeleopNetApp" {
  metadata {
    name = "TeleopNetApp"
    namespace = "Add the NetApp namespace"
    labels = {
      app = "TeleopApp"
    }
  }

  spec {
    container {
      image = "dockerhub.hi.inet/evolved-5g/TeleopNetApp:latest"
      name  = "TeleopContainer"
    }
  }
}

resource "kubernetes_service" "TeleopNetApp_service" {
  metadata {
    name = "TeleopService"
    namespace = "Add the NetApp namespace"
  }
  spec {
    selector = {
      app = Add the NetApp service app
    }
    port {
      port = 1191
      target_port = 1191
    }
  }
}
