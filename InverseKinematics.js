class InverseKinematics {
    constructor(){

        this.dhd0 = 105;
        this.dhd1 = 165;
        this.dhd2 = 0;
        this.dhd3 = 0;
        this.dhd4 = 400;
        this.dhd5 = 0;
        this.dhd6 = 90;

        this.dha2 = 200;
        this.dha3 = 355;
        this.dha4 = 60;
        this.dha5 = 0;
        this.dha6 = 0;

        this.matX2 = new THREE.Matrix4();
        this.matX3 = new THREE.Matrix4();
        this.matX4 = new THREE.Matrix4();
        this.matX5 = new THREE.Matrix4();
        this.matX6 = new THREE.Matrix4();
        this.matX2.set(
            1,0,0,this.dha2,
            0,0,-1,0,
            0,1,0,0,
            0,0,0,1);
        this.matX3.set(
            1,0,0,this.dha3,
            0,1,0,0,
            0,0,1,0,
            0,0,0,1);
        this.matX4.set(
            1,0,0,this.dha4,
            0,0,-1,0,
            0,1,0,0,
            0,0,0,1);
        this.matX5.set(
            1,0,0,this.dha5,
            0,0,1,0,
            0,-1,0,0,
            0,0,0,1);
        this.matX6.set(
            1,0,0,this.dha6,
            0,0,-1,0,
            0,1,0,0,
            0,0,0,1);

        this.matZ0 = new THREE.Matrix4();
        this.matZ0.set(1,0,0,0,
            0,1,0,0,
            0,0,1,this.dhd0,
            0,0,0,1);
        this.matZ1 = new THREE.Matrix4();
        this.matZ2 = new THREE.Matrix4();
        this.matZ3 = new THREE.Matrix4();
        this.matZ4 = new THREE.Matrix4();
        this.matZ5 = new THREE.Matrix4();
        this.matZ6 = new THREE.Matrix4();
    }

    setNowJointValue(val1, val2, val3, val4, val5, val6){
        this.SetTheta1 = val1;
        this.SetTheta2 = val2;
        this.SetTheta3 = val3;
        this.SetTheta4 = val4;
        this.SetTheta5 = val5;
        this.SetTheta6 = val6;
    }

    set SetTheta1(value){
        this.matZ1.set(
            Math.cos(value), -Math.sin(value), 0, 0,
            Math.sin(value), Math.cos(value), 0, 0,
            0, 0, 1, this.dhd1,
            0,0,0,1
        );
    }
    set SetTheta2(value){
        this.matZ2.set(
            Math.cos(value), -Math.sin(value), 0, 0,
            Math.sin(value), Math.cos(value), 0, 0,
            0, 0, 1, this.dhd2,
            0,0,0,1
        );
    }
    set SetTheta3(value){
        this.matZ3.set(
            Math.cos(value), -Math.sin(value), 0, 0,
            Math.sin(value), Math.cos(value), 0, 0,
            0, 0, 1, this.dhd3,
            0,0,0,1
        );
    }
    set SetTheta4(value){
        this.matZ4.set(
            Math.cos(value), -Math.sin(value), 0, 0,
            Math.sin(value), Math.cos(value), 0, 0,
            0, 0, 1, this.dhd4,
            0,0,0,1
        );
    }
    set SetTheta5(value){
        this.matZ5.set(
            Math.cos(value), -Math.sin(value), 0, 0,
            Math.sin(value), Math.cos(value), 0, 0,
            0, 0, 1, this.dhd5,
            0,0,0,1
        );
    }
    set SetTheta6(value){
        this.matZ6.set(
            Math.cos(value), -Math.sin(value), 0, 0,
            Math.sin(value), Math.cos(value), 0, 0,
            0, 0, 1, this.dhd6,
            0,0,0,1
        );
    }

    CalcForwardKinematics(){
        let resMat = new THREE.Matrix4();
        resMat = resMat.multiply(this.matZ0);
        resMat = resMat.multiply(this.matZ1);
        resMat = resMat.multiply(this.matX2);
        resMat = resMat.multiply(this.matZ2);
        resMat = resMat.multiply(this.matX3);
        resMat = resMat.multiply(this.matZ3);
        resMat = resMat.multiply(this.matX4);
        resMat = resMat.multiply(this.matZ4);
        resMat = resMat.multiply(this.matX5);
        resMat = resMat.multiply(this.matZ5);
        resMat = resMat.multiply(this.matX6);
        resMat = resMat.multiply(this.matZ6);

        return resMat;
    }

    CalcInverseKinematics(aTgtMat){

        let tgtPos = new Vector3().setFromMatrixPosition(aTgtMat);
        let tarZDir = new Vector3().setFromMatrixColumn(aTgtMat, 3);
    }
}