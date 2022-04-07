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

		stage('packagecloud'){
			steps {
			}
		}
	}

}
