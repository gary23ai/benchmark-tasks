/** Build the unpowered inspection replica from the same millimetre geometry as the simulator.
 * Run in this directory with Node 20+ and manifold-3d 3.5.5 (see BUILD.md).
 */
import fs from 'node:fs';
import path from 'node:path';
import {fileURLToPath} from 'node:url';
const moduleFactory = (await import(process.env.MIXER_CAD_MODULE || 'manifold-3d')).default;
const kernel = await moduleFactory();
kernel.setup();
const {Manifold: M} = kernel;
const here = path.dirname(fileURLToPath(import.meta.url));
const spec = JSON.parse(fs.readFileSync(path.join(here, '../task_impl/fixture.json'), 'utf8'));
const actors = Object.fromEntries(spec.actors.map(a => [a.name, a]));
const mm = v => v.map(x => Math.round(x * 1e7) / 1e4);
const box = (lo, hi) => M.cube(hi.map((v,i)=>v-lo[i])).translate(lo);
const union = solids => M.union(solids);
const cyl = (diam, height) => M.cylinder(height, diam/2, diam/2, 64, true);
const boreZ = (x,y,z0,z1,d) => cyl(d,z1-z0).translate([x,y,(z1+z0)/2]);
const boreX = (x0,x1,y,z,d) => cyl(d,x1-x0).rotate([0,90,0]).translate([(x0+x1)/2,y,z]);
function primitive(p) {
    const [x,y,z] = mm(p.size);
    let solid;
    if (p.type === 'box') solid = M.cube([x,y,z],true);
    else if (p.type === 'cylinder') solid = cyl(x,z);
    else if (p.type === 'sphere') solid = M.sphere(x/2,48);
    else if (p.type === 'capsule') {
        const cap = M.sphere(x/2,64);
        solid = M.hull([cap.translate([0,0,(z-x)/2]),cap.translate([0,0,-(z-x)/2])]);
    } else if (p.type === 'wedge') {
        const points=[];
        for (const u of [-x/2,x/2]) for (const v of [-y/2,y/2]) {
            points.push([u,v,-z/2],[u,v,v<0 ? z/2 : -z/2+p.toe_m*1000]);
        }
        solid=M.hull(points);
    } else throw new Error('Unknown CAD primitive '+p.type);
    return solid.rotate(p.euler_deg).translate(mm(p.offset));
}
const nativeParts = (name,indices) => (indices || actors[name].parts.map((_,i)=>i)).map(i=>primitive(actors[name].parts[i]));
const cradle = actors.cradle;
const rearBracket = nativeParts('return_bracket').map(p=>p.translate([150,0,0]));
const mountPattern = [[163,-21],[163,21],[179,-21],[179,21]];
const mountHoles = union(mountPattern.map(([x,y])=>boreZ(x,y,-45,-22,4.4)));
const plate = box([106,-25,-29.6],[193,25,-25.5]);
const pocket = nativeParts('cradle',Array.from({length:15},(_,i)=>i+12));
const bridgeDeck = box([145.2,-11,21.35],[154.8,11,24]);
const antiRollSill = box([145.2,-9,-13.2],[154.8,9,6.65]);
const keyLegs = [-1,1].map(s => box([145.2,s<0?-13.6:7.2,-13.2],[154.8,s<0?-7.2:13.6,23]));
const towerBolts=union([-17,17].map(y=>boreZ(189.5,y,-32,-18,3.4)));
const bushTower = union([box([187.6,-12,-25.5],[192,12,24]),box([186,-20.5,-25.5],[193,20.5,-21.5])]).subtract(boreX(185,195,0,14,8.05)).subtract(towerBolts);
let cassette=union([plate,...pocket,...rearBracket,bridgeDeck,antiRollSill,...keyLegs]).subtract(mountHoles).subtract(towerBolts);
let slider=union(nativeParts('catch',[0,1,2])).subtract(boreX(145.6,160.5,0,14,6.5));
const steelStem=boreX(145.6,182,0,14,6.4);
const bush=boreX(175.6,192,0,14,8).subtract(boreX(175,193,0,14,6.65));
const fullPins=[...nativeParts('cradle',[8,9,27,28,29,30,31,32,33,34,35,36,37,38])];
const pin = union([...fullPins,boreZ(0,123.2,-24,-8.5,6.4)]).translate([0,-123.2,0]);
let shoulders=[];
for (const s of [-1,1]) {
    const cy=s*123.2;
    for (const offset of [-14.4,14.4]) {
        const y=cy+offset;
        shoulders.push(cyl(12.8,3.2).rotate([90,0,0]).translate([0,y,-6.4]));
        shoulders.push(box([-4.5,y-1.6,-13.2],[4.5,y+1.6,-9.5]));
    }
}
const receiverCut=box([105.7,-25.3,-29.75],[193.3,25.3,27]);
const armBores=union([-123.2,123.2].map(y=>boreZ(0,y,-24.1,-7,6.55)));
const staticNames=['machine_base','motor_housing','drive_hub','lift_handle'];
const internalBraces=[box([158,-35,40],[192,35,141]),box([188,-8,105],[214,8,138]),boreZ(0,0,144,149,15)];
const benchBolts=union([[135,-119],[135,119],[193,-119],[193,119]].map(([x,y])=>boreZ(x,y,-131,-110,6.6)));
const upperCradle=box([156.4,-57.6,28.8],[194.8,57.6,142.4]);
const cradleLegs=[box([156.4,-57.6,-115.2],[194.8,-28.8,29]),box([156.4,28.8,-115.2],[194.8,57.6,29])];
const travelStop=union([box([194.8,-29,-12.8],[207.6,29,29]),box([194.5,-29.1,28.6],[196,29.1,29.3]),box([194.5,-29.1,-12.7],[196,-28.6,29.2]),box([194.5,28.6,-12.7],[196,29.1,29.2])]);
let chassis=union([...nativeParts('cradle',[0,5,6,7]),travelStop,upperCradle,...cradleLegs,...shoulders,
    ...staticNames.flatMap(n=>nativeParts(n)),...internalBraces]).subtract(receiverCut).subtract(mountHoles).subtract(armBores).subtract(benchBolts);
// Three slip-fit dowels align the split tower. The split lies in solid column sections.
const jointHoles=union([[175.6,-43.2],[175.6,43.2],[237,0]].map(([x,y])=>boreZ(x,y,83,107,6.15)));
chassis=chassis.subtract(jointHoles);
const lower=chassis.intersect(box([-300,-300,-150],[400,300,95]));
const upper=chassis.intersect(box([-300,-300,95],[400,300,350]));
const nativeBowl=nativeParts('bowl');
const bowlSkin=M.hull(nativeBowl.slice(0,121));
const shelves=[45,135,225,315].map(angle => box([66,-7,-24],[91,7,-21]).rotate([0,0,angle]).intersect(bowlSkin));
const bowl=union([...nativeBowl,...shelves]);
const out = process.argv[2] || path.join(here,'parts');
fs.mkdirSync(out,{recursive:true});
function save(name,solid) {
    solid=solid.simplify(0.005);
    if (solid.status() !== 'NoError' || solid.isEmpty() || solid.decompose().length!==1) throw new Error('Invalid or disconnected '+name);
    const mesh=solid.getMesh(), verts=mesh.vertProperties, tri=mesh.triVerts, stride=mesh.numProp;
    const count=tri.length/3, buf=Buffer.alloc(84+50*count);
    buf.write('Inspection fixture | millimetres | '+name,0,80,'ascii');buf.writeUInt32LE(count,80);
    for(let i=0;i<count;i++){
        const v=[0,1,2].map(j=>[0,1,2].map(k=>verts[tri[3*i+j]*stride+k]));
        const ab=v[1].map((t,k)=>t-v[0][k]),ac=v[2].map((t,k)=>t-v[0][k]);
        const n=[ab[1]*ac[2]-ab[2]*ac[1],ab[2]*ac[0]-ab[0]*ac[2],ab[0]*ac[1]-ab[1]*ac[0]];
        const norm=Math.hypot(...n)||1;
        const vals=[...n.map(t=>t/norm),...v.flat()];vals.forEach((t,j)=>buf.writeFloatLE(t,84+i*50+4*j));
    }
    fs.writeFileSync(path.join(out,name+'.stl'),buf);
    return {name,triangles:count,bounds:solid.boundingBox(),volumeMm3:solid.volume(),components:solid.decompose().length};
}
const pieces={pocket_cassette:cassette,guide_tower:bushTower,sliding_catch:slider,cradle_and_column:lower,head_and_upper_column:upper,bowl_and_fittings:bowl,locator_pin_profile:pin};
const report=Object.entries(pieces).map(([name,solid])=>save(name,solid));
const checks=[];
for (let i=0;i<=128;i++){
    const dx=i/10,unit=union([slider,steelStem]).translate([dx,0,0]);
    for(const [name,obstacle] of Object.entries({cassette,bush,tower:bushTower,chassis})){
        const interference=unit.intersect(obstacle).volume();
        if(interference>0.01) throw new Error('Interference at '+dx+' mm in '+name+': '+interference+' mm3');
    }
    if(i%32===0) checks.push({travelMm:dx,modelSpringN:2.56+0.2*dx,
        rodBushingOverlapMm:Math.min(182+dx,192)-175.6,rodToStopMm:194.8-(182+dx)});
}
const stud=primitive(actors.bowl.parts[147]);
const retention={seatedOverlapMm3:stud.intersect(slider).volume(),
    lifted3mmBlockedMm3:stud.translate([0,0,3]).intersect(slider).volume(),
    retractedLiftMaxOverlapMm3:Math.max(...Array.from({length:81},(_,i)=>stud.translate([0,0,i/2]).intersect(slider.translate([12.8,0,0])).volume())),
    forwardStopOverrunMm3:slider.translate([-.1,0,0]).intersect(cassette).volume(),
    rearStopOverrunMm3:steelStem.translate([12.9,0,0]).intersect(chassis).volume(),
    fixedCassetteMisfitMm3:cassette.intersect(chassis).volume(),
    towerMisfitMm3:bushTower.intersect(chassis).volume()};
if (retention.seatedOverlapMm3>0.01 || retention.retractedLiftMaxOverlapMm3>0.01 || retention.lifted3mmBlockedMm3<1 ||
    retention.forwardStopOverrunMm3<.1 || retention.rearStopOverrunMm3<.1 ||
    retention.fixedCassetteMisfitMm3>.01 || retention.towerMisfitMm3>.01) throw new Error('Retention or assembly check failed '+JSON.stringify(retention));
fs.writeFileSync(path.join(out,'geometry-check.json'),JSON.stringify({pieces:report,travelChecks:checks,retention},null,2)+'\n');
console.log(report);console.log(checks);
