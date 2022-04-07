pipeline {
	agent any

	stages {
		stage('do tha build'){
			steps{
				cleanWs()

				fileOperations([folderCreateOperation('source')])
				dir('source'){
					checkout scm
				}

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
				rtBuildInfo (
    // Optional - Maximum builds to keep in Artifactory.
    maxBuilds: 1,
				)

			rtPublishBuildInfo (
    serverId: 'rm5248-jfrog'
			)

				rtUpload (
    serverId: 'rm5248-jfrog',
    spec: '''{
          "files": [
            {
              "pattern": "binaries*/*",
              "target": "test-repo-debian-local/pool/cserial/",
              "props":"deb.distribution=buster;deb.component=main;deb.architecture=amd64"
            }
         ]
    }''' )
			}
		}
	}

}
