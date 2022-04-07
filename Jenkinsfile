pipeline {
	agent any

	stages {
		stage('do tha build'){
			steps{
				cleanWs()

				checkout scm

				debianPbuilder additionalBuildResults: '', 
					architecture: '', 
					components: '', 
					distribution: 'buster', 
					keyring: '', 
					mirrorSite: 'http://deb.debian.org/debian', 
					pristineTarName: ''

				fingerprint 'binaries/*.deb'
			}
		}

		stage('jfrog'){
			steps {
				rtUpload (
    serverId: 'rm5248-jfrog',
    spec: '''{
          "files": [
            {
              "pattern": "binaries*/*",
              "target": "test-repo-debian-local/cserial/"
            }
         ]
    }''' )
			}
		}
	}

}
