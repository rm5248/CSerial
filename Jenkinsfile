#!/usr/bin/env groovy

def axisArchitecture = ["amd64", "armhf"]
def axisNode = ["master"]
def tasks = [:]

for( int i = 0; i < axisArchitecture.size(); i++ ){
    def arch = axisArchitecture[i];
    tasks["${axisNode[0]}/${axisArchitecture[i]}"] = {
        node(axisNode[0]){
            ws{
		stage( "checkout" ){
			checkout scm
		}
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

