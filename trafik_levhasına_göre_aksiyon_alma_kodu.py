import lanelet2
import lanelet2.io
import lanelet2.projection
import lanelet2.traffic_rules
import lanelet2.routing

# OSM dosya yolu
filename = "/home/emirhan/Documents/simulationitrack_main_correct_new.3.osm"
latitude = 0.0
longitude = 0.0
origin = lanelet2.io.Origin(latitude, longitude)
projector = lanelet2.projection.LocalCartesianProjector(origin)

# Haritayı yükle ve routing graph oluştur
map = lanelet2.io.load(filename, projector)
traffic_rules = lanelet2.traffic_rules.create(
    lanelet2.traffic_rules.Locations.Germany,
    lanelet2.traffic_rules.Participants.Vehicle,
)
routing_graph = lanelet2.routing.RoutingGraph(map, traffic_rules)

print("Harita başarıyla yüklendi!")
print("Lanelet sayısı:", len(map.laneletLayer))
print("Linestring sayısı:", len(map.lineStringLayer))
print("Nokta sayısı:", len(map.pointLayer))

# Kullanıcıdan veri al
detected_sign = input("Tespit edilen levhayı girin (örnek: saga donulmez / sola mecburi yon / Ileriden saga mecburi yon): ").strip()
lanelet_id = int(input("Mevcut lanelet ID'sini girin: ").strip())

if lanelet_id not in map.laneletLayer:
    raise ValueError(f"Lanelet ID {lanelet_id} haritada bulunamadı.")

current_lanelet = map.laneletLayer[lanelet_id]

# Follower’ları bul
followers = routing_graph.following(current_lanelet)
if not followers:
    print(f"Lanelet {lanelet_id} için takip edilebilecek lanelet yok.")
    exit(0)

allowed = []
blocked = []
yasakli_lanelet_idler = []

# turn_direction listesi
turn_dirs = [f.attributes["turn_direction"] if "turn_direction" in f.attributes else "belirtilmemiş" for f in followers]

# Levha türüne göre yön kontrolü
def relevant_direction_exists(sign, directions):
    if sign == "saga donulmez":
        return "right" in directions
    elif sign == "sola donulmez":
        return "left" in directions
    elif sign == "sola mecburi yon":
        return "left" in directions
    elif sign == "saga mecburi yon":
        return "right" in directions
    elif sign == "Ileri mecburi yon":
        return "straight" in directions
    elif sign == "Ileri ve saga mecburi yon":
        return "straight" in directions or "right" in directions
    elif sign == "Ileri ve sola mecburi yon":
        return "straight" in directions or "left" in directions
    elif sign in ["Ileriden sola mecburi yon", "Ileriden saga mecburi yon"]:
        return "straight" in directions
    return False

if not relevant_direction_exists(detected_sign, turn_dirs):
    print(f"Uyarı: '{detected_sign}' levhası için uygun yön bulunamadı. Levha dikkate alınmadı.")
    detected_sign = None  # Levha yokmuş gibi davran

for nxt in followers:
    td = nxt.attributes["turn_direction"] if "turn_direction" in nxt.attributes else "belirtilmemiş"
    block = False

    if detected_sign == "saga donulmez":
        if td == "right":
            block = True
    elif detected_sign == "sola donulmez":
        if td == "left":
            block = True
    elif detected_sign == "sola mecburi yon":
        if td != "left":
            block = True
    elif detected_sign == "saga mecburi yon":
        if td != "right":
            block = True
    elif detected_sign == "Ileri mecburi yon":
        if td != "straight":
            block = True
    elif detected_sign == "Ileri ve saga mecburi yon":
        if td == "left":
            block = True
    elif detected_sign == "Ileri ve sola mecburi yon":
        if td == "right":
            block = True
    elif detected_sign == "Ileriden sola mecburi yon":
        if td == "straight":
            straight_lanelet = nxt
            second_followers = routing_graph.following(straight_lanelet)
            if not any("left" == (sec.attributes["turn_direction"] if "turn_direction" in sec.attributes else "") for sec in second_followers):
                print("Uyarı: İleriden sola mecburi yön levhası için sola giden yol bulunamadı.")
                continue
            for sec in second_followers:
                sec_td = sec.attributes["turn_direction"] if "turn_direction" in sec.attributes else "belirtilmemiş"
                if sec_td != "left":
                    blocked.append((sec.id, sec_td))
                    yasakli_lanelet_idler.append(sec.id)
                else:
                    allowed.append((sec.id, sec_td))
    elif detected_sign == "Ileriden saga mecburi yon":
        if td == "straight":
            straight_lanelet = nxt
            second_followers = routing_graph.following(straight_lanelet)
            if not any("right" == (sec.attributes["turn_direction"] if "turn_direction" in sec.attributes else "") for sec in second_followers):
                print("Uyarı: İleriden sağa mecburi yön levhası için sağa giden yol bulunamadı.")
                continue
            for sec in second_followers:
                sec_td = sec.attributes["turn_direction"] if "turn_direction" in sec.attributes else "belirtilmemiş"
                if sec_td != "right":
                    blocked.append((sec.id, sec_td))
                    yasakli_lanelet_idler.append(sec.id)
                else:
                    allowed.append((sec.id, sec_td))
    else:
        continue  # Geçersiz ya da dikkate alınmayan levha

    if detected_sign not in ["Ileriden sola mecburi yon", "Ileriden saga mecburi yon"]:
        if block:
            blocked.append((nxt.id, td))
            yasakli_lanelet_idler.append(nxt.id)
        else:
            allowed.append((nxt.id, td))

# Sonuçları yazdır
print(f"\nLevha: {detected_sign if detected_sign else 'YOK (yön bulunamadı)'}")
print(f"Mevcut lanelet ID: {lanelet_id}")

print("\n→ İzin verilen follower lanelet'ler:")
for ll_id, td in allowed:
    print(f"   • ID: {ll_id}, turn_direction: {td}")

print("\n→ Bloklanan follower lanelet'ler:")
for ll_id, td in blocked:
    print(f"   • ID: {ll_id}, turn_direction: {td}")

print("\n→ Yasaklı lanelet ID listesi:")
print(yasakli_lanelet_idler)


