#!/usr/bin/env groovy

def axisArchitecture = ["amd64", "armhf"]
def axisNode = ["master"]
def tasks = [:]

for( int i = 0; i < axisArchitecture.size(); i++ ){
    def arch = axisArchitecture[i];
    tasks["${axisNode[0]}/${axisArchitecture[i]}"] = {
        node(axisNode[0]){
	stage( "checkout" ){
checkout([$class: 'GitSCM', branches: [[name: '*/jenkinsfile-updates']], doGenerateSubmoduleConfigurations: false, extensions: [[$class: 'RelativeTargetDirectory', relativeTargetDir: 'source']], submoduleCfg: [], userRemoteConfigs: [[credentialsId: '78de5c66-5dfb-4c95-8ad9-ec34e8dee4ec', url: 'git@github.com:rm5248/CSerial.git']]])
   	}
            ws{
                stage( "clean" ){
                    cleanWs()
                }
                stage("build-${arch}"){
                    debianPbuilder architecture:"${arch}"
                }
            }
        }
    }
}

stage('build'){
    parallel tasks
}

